/*
 * sfhss.c
 *
 *  Created on: 2019/01/20
 *      Author: opiopan@gmail.com
 */

#include <string.h>
#include "sfhss.h"
#include "log.h"

#define SFHSS_ENABLE_DMA_READFIFO
//#define SFHSS_ENABLE_DMA_HOPPING
//#define SFHSS_ENABLE_DMA_AHOPPING

#define BINDING_MEASURE_COUNT 15
#define SHORT_INTERVAL_MAX (6801 + 612)
#define SHORT_INTERVAL_MIN (6801 - 612)
#define LONG_INTERVAL_MAX (6801 * 30 + 612)
#define LONG_INTERVAL_MIN (6801 * 30 - 612)
#define HOPPING_TIMEOUT 200
#define FALLBACK_COUNT 30

#define PACKET_LENGTH 15

// Some important initialization parameters, all others are either default,
// IOCFG2   07 - GDO2_INV=0 GDO2_CFG=07 - assert when packet is received
// IOCFG1   2E - GDO1_INV=0 GDO1_CFG=2E - High Impedance
// IOCFG0   3F - GDO0_INV=0 GDO0_CFG=3F - CLK XOSC/192
// FIFOTHR  07 - 33 decimal RX FIFO threshold
// SYNC1    D3
// SYNC0    91
// PKTLEN   0D - Packet length, 0D bytes
// PKTCTRL1 04 - APPEND_STATUS on, all other are receive parameters - irrelevant
// PKTCTRL0 0C - No whitening, use FIFO, CC2400 compatibility on, use CRC, fixed packet length
// ADDR     29
// CHANNR   10
// FSCTRL1  06 - IF 152343.75Hz, see page 65
// FSCTRL0  00 - zero freq offset
// FREQ2    5C - synthesizer frequency 2399999633Hz for 26MHz crystal, ibid
// FREQ1    4E
// FREQ0    C4
// MDMCFG4  7C - CHANBW_E - 01, CHANBW_M - 03, DRATE_E - 0C. Filter bandwidth = 232142Hz
// MDMCFG3  43 - DRATE_M - 43. Data rate = 128143bps
// MDMCFG2  83 - disable DC blocking, 2-FSK, no Manchester code, 15/16 sync bits detected
// MDMCFG1  23 - no FEC, 4 preamble bytes, CHANSPC_E - 03
// MDMCFG0  3B - CHANSPC_M - 3B. Channel spacing = 249938Hz (each 6th channel used, resulting in spacing of 1499628Hz)
// DEVIATN  44 - DEVIATION_E - 04, DEVIATION_M - 04. Deviation = 38085.9Hz
// MCSM2    07 - receive parameters, default, irrelevant
// MCSM1    3C - default CCA, when packet received stay RX mode, when sent go to IDLE
// MCSM0    08 - no autocalibration, PO_TIMEOUT - 64, no pin radio control, no forcing XTAL to stay in SLEEP
// FOCCFG   1D - not interesting, Frequency Offset Compensation
// BSCFG    1C
// AGCCTRL2 43
// AGCCTRL1 40
// AGCCTRL1 91
// WOREVT1  57
// WOREVT0  6B
// WORCTRL  F8
// FREND1   B6
// FREND0   10 - PA_POWER = 0
// FSCAL3   EA
// FSCAL2   0A
// FSCAL1   11
// FSCAL0   11
const uint8_t SFHSS_init_values[] = {
  /* 00 */ 0x07, 0x2e, 0x3f, 0x07, 0xD3, 0x91, 0x0D, 0x04,
  /* 08 */ 0x0c, 0x29, 0x10, 0x06, 0x00, 0x5c, 0x4e, SFHSS_FREQ0_VAL + SFHSS_COARSE,
  /* 10 */ 0x7c, 0x43, 0x83, 0x23, 0x3b, 0x44, 0x07, 0x3c,
  /* 18 */ 0x08, 0x1d, 0x1c, 0x43, 0x40, 0x91, 0x57, 0x6b,
  /* 20 */ 0xf8, 0xb6, 0x10, 0xea, 0x0a, 0x11, 0x11
};

static void chuneChannel(SFHSSCTX* ctx)
{
    cc2500_beginMulitipleOps(ctx->cc2500);
    cc2500_addStrobeOps(ctx->cc2500, CC2500_SIDLE);
    cc2500_addWriteRegisterOps(ctx->cc2500, CC2500_0A_CHANNR, SFHSS_CH(ctx));
    cc2500_addStrobeOps(ctx->cc2500, CC2500_SCAL);
    cc2500_commitMultipleOps(ctx->cc2500);
}

static void chuneChannelFast(SFHSSCTX *ctx)
{
    cc2500_beginMulitipleOps(ctx->cc2500);
    cc2500_addWriteRegisterOps(ctx->cc2500, CC2500_25_FSCAL1, SFHSS_CALDATA(ctx));
    cc2500_addWriteRegisterOps(ctx->cc2500, CC2500_0A_CHANNR, SFHSS_CH(ctx));
    cc2500_commitMultipleOps(ctx->cc2500);
}

#ifdef SFHSS_ENABLE_DMA_HOPPING
static void chuneChannelFastDMA(SFHSSCTX* ctx)
{
    cc2500_beginMulitipleOps(ctx->cc2500);
    cc2500_addWriteRegisterOps(ctx->cc2500, CC2500_25_FSCAL1, SFHSS_CALDATA(ctx));
    cc2500_addWriteRegisterOps(ctx->cc2500, CC2500_0A_CHANNR, SFHSS_CH(ctx));
    cc2500_issueDMAforMultipleOps(ctx->cc2500);
}
#endif

#ifndef SFHSS_ENABLE_DMA_AHOPPING
static void chuneChannelFastWithFIFOFlash(SFHSSCTX *ctx)
{
    cc2500_beginMulitipleOps(ctx->cc2500);
    cc2500_addStrobeOps(ctx->cc2500, CC2500_SIDLE);
    cc2500_addWriteRegisterOps(ctx->cc2500, CC2500_25_FSCAL1, SFHSS_CALDATA(ctx));
    cc2500_addWriteRegisterOps(ctx->cc2500, CC2500_0A_CHANNR, SFHSS_CH(ctx));
    cc2500_addStrobeOps(ctx->cc2500, CC2500_SFRX);
    cc2500_addStrobeOps(ctx->cc2500, CC2500_SRX);
    cc2500_commitMultipleOps(ctx->cc2500);
}

#else
static void chuneChannelFastWithFIFOFlashDMA(SFHSSCTX* ctx)
{
    cc2500_beginMulitipleOps(ctx->cc2500);
    cc2500_addStrobeOps(ctx->cc2500, CC2500_SIDLE);
    cc2500_addWriteRegisterOps(ctx->cc2500, CC2500_25_FSCAL1, SFHSS_CALDATA(ctx));
    cc2500_addWriteRegisterOps(ctx->cc2500, CC2500_0A_CHANNR, SFHSS_CH(ctx));
    cc2500_addStrobeOps(ctx->cc2500, CC2500_SFRX);
    cc2500_addStrobeOps(ctx->cc2500, CC2500_SRX);
    cc2500_issueDMAforMultipleOps(ctx->cc2500);
}
#endif

#if defined(SFHSS_ENABLE_DMA_HOPPING) || defined(SFHSS_ENABLE_DMA_AHOPPING)
static void completeChuneChannelDMA(SFHSSCTX* ctx){
    cc2500_commitMultipleOps(ctx->cc2500);
}
#endif

static void nextChannel(SFHSSCTX* ctx, uint8_t hopcode)
{
    uint8_t ch = ctx->ch;
    ch += hopcode + 2;
    if (ch > 29)
    {
        if (ch < 31)
            ch += hopcode + 2;
        ch -= 31;
    }
    ctx->ch = ch;
}

static bool readPacket(SFHSSCTX *ctx)
{
    cc2500_beginMulitipleOps(ctx->cc2500);
    ctx->dmabuf = cc2500_addReadFIFOOps(ctx->cc2500, PACKET_LENGTH);
    return cc2500_commitMultipleOps(ctx->cc2500);
}

#ifdef SFHSS_ENABLE_DMA_READFIFO
static bool readPacketDMA(SFHSSCTX *ctx)
{
    cc2500_beginMulitipleOps(ctx->cc2500);
    ctx->dmabuf = cc2500_addReadFIFOOps(ctx->cc2500, PACKET_LENGTH);
    return cc2500_issueDMAforMultipleOps(ctx->cc2500);
}

static bool completeReadPacketDMA(SFHSSCTX *ctx)
{
    return cc2500_commitMultipleOps(ctx->cc2500);
}
#endif

static bool parsePacket(SFHSSCTX *ctx, uint8_t *cmd)
{
    SFHSS_RESET_RECEIVED(ctx);

    int rc = ctx->dmabuf[0];
    if (rc & CC2500_STATUS_CHIP_RDYn_BM){
        LOG_LOGW("SFHSS: failed packet recieve [%x]", rc);
        return false;
    }
    if ((rc & CC2500_STATUS_FIFO_BYTES_AVAILABLE_BM) < PACKET_LENGTH){
        LOG_LOGE("SFHSS: insufficient packet data [%d]", rc & CC2500_STATUS_FIFO_BYTES_AVAILABLE_BM);
        return false;
    }

    uint8_t *pkt = ctx->dmabuf + 1;
    uint16_t txaddr = (uint16_t)pkt[1] << 8 | pkt[2];
    uint8_t hopcode = ((pkt[11] & 0x7) << 2) | ((pkt[12]  & 0xc0) >> 6);
    *cmd = pkt[12] & 0x3f;

    if (ctx->txaddr < 0){
        ctx->txaddr = txaddr;
    }
    if (ctx->txaddr != txaddr){
        return false;
    }
    ctx->hopcode = hopcode;

    uint16_t data[4];
    data[0] = (((uint16_t)pkt[5] & 0x07) << 9) | ((uint16_t)pkt[6] << 1) | (((uint16_t)pkt[7] & 0x80) >> 7);
    data[1] = (((uint16_t)pkt[7] & 0x7f) << 5) | (((uint16_t)pkt[8] & 0xf8) >> 3);
    data[2] = (((uint16_t)pkt[8] & 0x07) << 9) | ((uint16_t)pkt[9] << 1) | (((uint16_t)pkt[10] & 0x80) >> 7);
    data[3] = (((uint16_t)pkt[10] & 0x7f) << 5) | (((uint16_t)pkt[11] & 0xf8) >> 3);

    if (*cmd & 4){
        ctx->stat.failsafe++;
    }else{
        int offset = (*cmd & 1) << 2;
        ctx->isDirty |=
                data[0] != ctx->data[offset + 0] ||  data[1] != ctx->data[offset + 1] ||
                data[2] != ctx->data[offset + 2] || data[3] != ctx->data[offset + 3];
        ctx->data[offset + 0] = data[0];
        ctx->data[offset + 1] = data[1];
        ctx->data[offset + 2] = data[2];
        ctx->data[offset + 3] = data[3];
    }

    if (ctx->phase == SFHSS_BINDING){
        if (ctx->measureCount[ctx->packetPos] > 0){
            ctx->interval[ctx->packetPos] = ctx->rtime - ctx->ptime[ctx->packetPos];
            ctx->intervalSum[ctx->packetPos] += ctx->interval[ctx->packetPos];
        }
        ctx->measureCount[ctx->packetPos] += 1;
    }
    ctx->ptime[ctx->packetPos] = ctx->rtime;
    ctx->packetPos = 1 - (*cmd & 1);

    return true;
}

void sfhss_init(SFHSSCTX* ctx, CC2500CTX* cc2500)
{
    *ctx = (SFHSSCTX){
        .cc2500 = cc2500,
        .phase = SFHSS_INIT,
        .data = {
            SFHSS_CENTERPOS_WIDTH,
            SFHSS_CENTERPOS_WIDTH,
            0,
            SFHSS_CENTERPOS_WIDTH,
            SFHSS_CENTERPOS_WIDTH,
            SFHSS_CENTERPOS_WIDTH,
            SFHSS_CENTERPOS_WIDTH,
            SFHSS_CENTERPOS_WIDTH,
        }};
    cc2500_writeRegisterBurst(ctx->cc2500, 0, SFHSS_init_values, sizeof(SFHSS_init_values));
}

void sfhss_calibrate(SFHSSCTX* ctx)
{
    for (int i = 0; i < SFHSS_CHNUM; i++){
        ctx->ch = i;
        chuneChannel(ctx);
        cc2500_waitForState(ctx->cc2500, CC2500_STATE_IDLE);
        cc2500_readRegister(ctx->cc2500, CC2500_25_FSCAL1, &SFHSS_CALDATA(ctx));
    }
    ctx->phase = SFHSS_CALIBRATED;
    LOG_LOGI("SFHSS: retrieved %d ch calibrate data", sizeof(ctx->caldata));
    log_dumpmem(ctx->caldata, sizeof(ctx->caldata));
}

SFHSS_EVENT sfhss_schedule(SFHSSCTX* ctx, int32_t now)
{
    SFHSS_EVENT rc = SFHSSEV_NONE;

    switch ((int)ctx->phase){

    case SFHSS_CALIBRATED:{
        cc2500_strobe(ctx->cc2500, CC2500_SIDLE);
        cc2500_strobe(ctx->cc2500, CC2500_SFRX);
        ctx->ch = 0;
        chuneChannelFast(ctx);
        SFHSS_RESET_RECEIVED(ctx);
        cc2500_strobe(ctx->cc2500, CC2500_SRX);
        ctx->phase = SFHSS_START_BINDING;
        LOG_LOGD("SFHSS: change status to START-BINDING");
        break;
    }

    case SFHSS_START_BINDING:{
        ctx->txaddr = -1;
        ctx->measureCount[0] = 0;
        ctx->measureCount[1] = 0;
        ctx->interval[0] = 0;
        ctx->interval[1] = 0;
        ctx->intervalSum[0] = 0;
        ctx->intervalSum[1] = 0;
        ctx->phase = SFHSS_FINDING_RADIO;
        rc = SFHSSEV_START_FINDING;
        LOG_LOGD("SFHSS: change status to FINDING-RADIO");
        break;
    }

    case SFHSS_FINDING_RADIO:{
        if (ctx->received){
            ctx->phase = SFHSS_BINDING;
            rc = SFHSSEV_START_BINDING;
            LOG_LOGD("SFHSS: change status to BINDING");
            LOG_LOGI("SFHSS: found radio");
        }
        break;
    }

    case SFHSS_BINDING:{
        if (ctx->received){
            uint8_t cmd;
            if (!readPacket(ctx) || !parsePacket(ctx, &cmd)){
                break;
            }
            if (ctx->measureCount[cmd & 1] > 1){
                int interval = ctx->interval[cmd & 1];
                if (!(interval > LONG_INTERVAL_MIN && interval < LONG_INTERVAL_MAX)){
                    ctx->phase = SFHSS_START_BINDING;
                    LOG_LOGW("SFHSS: cannot bind with transmitter, retry to find radio");
                    LOG_LOGD("SFHSS: change status to START-BINDING");
                }
            }
            if (ctx->measureCount[0] > BINDING_MEASURE_COUNT &&
                ctx->measureCount[1] > BINDING_MEASURE_COUNT){
                ctx->interval[0] = ctx->intervalSum[0] / (ctx->measureCount[0] - 1) / 30;
                ctx->interval[1] = ctx->intervalSum[1] / (ctx->measureCount[1] - 1) / 30;
                ctx->phase = SFHSS_BINDED;
                LOG_LOGI("SFHSS: binded with transmitter [%.4X]", ctx->txaddr);
                LOG_LOGD("SFHSS: change status to BINDED");
            }
        }else if (now - ctx->rtime > LONG_INTERVAL_MAX){
            ctx->phase = SFHSS_START_BINDING;
            LOG_LOGW("SFHSS: cannot bind with transmitter due to missed radio, retry to find radio");
            LOG_LOGD("SFHSS: change status to START-BINDING");
        }
        break;
    }

    case SFHSS_BINDED:{
        cc2500_strobe(ctx->cc2500, CC2500_SIDLE);
        cc2500_strobe(ctx->cc2500, CC2500_SFRX);
        ctx->ch = 0;
        chuneChannelFast(ctx);
        SFHSS_RESET_RECEIVED(ctx);
        cc2500_strobe(ctx->cc2500, CC2500_SRX);
        ctx->phase = SFHSS_CONNECTING1;
        rc = SFHSSEV_START_CONNECTING;
        LOG_LOGD("SFHSS: change status to CONNECTING1");
        break;
    }

    case SFHSS_CONNECTING1:{
        if (ctx->received){
            uint8_t cmd;
            readPacket(ctx);
            parsePacket(ctx, &cmd);
            if (!(cmd & 1)){
                ctx->phase = SFHSS_CONNECTING2;
                LOG_LOGD("SFHSS: change status to CONNECTING2");
            }
        }
        break;
    }

    case SFHSS_CONNECTING2:{
        if (ctx->received){
            uint8_t cmd;
            readPacket(ctx);
            parsePacket(ctx, &cmd);
            if (cmd & 1){
                if (ctx->ptime[1] - ctx->ptime[0] > ctx->interval[0] / 2){
                    ctx->phase = SFHSS_CONNECTING1;
                    LOG_LOGD("SFHSS: change status to CONNECTING1");
                }else{
                    ctx->phase = SFHSS_CONNECTED;
                    LOG_LOGI("SFHSS: connect to transmitter [%.4X]", ctx->txaddr);
                    LOG_LOGD("SFHSS: change status to CONNECTED");
                    nextChannel(ctx, ctx->hopcode);
                    chuneChannelFast(ctx);
                    rc = SFHSSEV_CONNECTED;
                }
            }
        }
        break;
    }

    case SFHSS_CONNECTED:{
        if (ctx->received){
            #ifdef SFHSS_ENABLE_DMA_READFIFO
            if (readPacketDMA(ctx)){
            #else
            if (readPacket(ctx)){
            #endif
                ctx->phase = SFHSS_PAKCET_RECEIVING;
            }else{
                ctx->phase = SFHSS_ABNORMAL_HOPPING;
            }
            ctx->stat.proc_rcv_cnt++;
            ctx->stat.proc_rcv_time += HRTIMER_GETTIME() - now;
        }else{
            int elapse = now - ctx->ptime[ctx->packetPos];
            if (elapse > (ctx->interval[ctx->packetPos] * (ctx->skipCount + 1)) + HOPPING_TIMEOUT){
                ctx->phase = SFHSS_ABNORMAL_HOPPING;
            }
        }
        break;
    }

    case SFHSS_PAKCET_RECEIVING:{
        if (!CC2500_DMA_IS_WORKING(ctx->cc2500)){
            #ifdef SFHSS_ENABLE_DMA_READFIFO
            if (!completeReadPacketDMA(ctx)){
                ctx->phase = SFHSS_ABNORMAL_HOPPING;
                break;
            }
            #endif
            uint8_t cmd;
            if (parsePacket(ctx, &cmd)){
                ctx->stat.rcv++;
                if (cmd & 1){
                    ctx->phase = SFHSS_HOPPING;
                }else{
                    ctx->phase = SFHSS_CONNECTED;
                }
                if (ctx->skipCount > 0){
                    LOG_LOGD("SFHSS: recover connection after skipping %d times", ctx->skipCount);
                    ctx->skipCount = 0;
                    ctx->ptime[((cmd & 1) + 1) & 1] = now;
                }
                ctx->stat.proc_endrcv_cnt++;
                ctx->stat.proc_endrcv_time += HRTIMER_GETTIME() - now;
            }else{
                ctx->phase = SFHSS_ABNORMAL_HOPPING;
            }
        }
        break;
    }

    case SFHSS_HOPPING:{
        nextChannel(ctx, ctx->hopcode);
        #ifdef SFHSS_ENABLE_DMA_HOPPING
        chuneChannelFastDMA(ctx);
        ctx->phase = SFHSS_COMPLETE_HOPPING;
        #else
        chuneChannelFast(ctx);
        ctx->phase = SFHSS_CONNECTED;
        #endif
        ctx->stat.proc_hopping_cnt++;
        ctx->stat.proc_hopping_time += HRTIMER_GETTIME() - now;
        break;
    }

    case SFHSS_ABNORMAL_HOPPING:{
        int skipnum = ctx->packetPos == 0 ? 2 : 1;
        ctx->skipCount++;
        if (ctx->skipCount < FALLBACK_COUNT){
            ctx->stat.lost += skipnum;
            ctx->packetPos = 0;
            nextChannel(ctx, ctx->hopcode);
            #ifdef SFHSS_ENABLE_DMA_AHOPPING
            chuneChannelFastWithFIFOFlashDMA(ctx);
            ctx->phase = SFHSS_COMPLETE_HOPPING;
            #else
            chuneChannelFastWithFIFOFlash(ctx);
            ctx->phase = SFHSS_CONNECTED;
            #endif
            ctx->stat.proc_ahopping_cnt++;
            ctx->stat.proc_ahopping_time += HRTIMER_GETTIME() - now;
        }else{
            ctx->phase = SFHSS_BINDED;
            LOG_LOGW("SFHSS: lost connection", ctx->txaddr);
            LOG_LOGD("SFHSS: change status to BINDED");
            rc = SFHSSEV_START_CONNECTING;
        }
        break;
    }

    case SFHSS_COMPLETE_HOPPING:{
        #if defined(SFHSS_ENABLE_DMA_HOPPING) || defined(SFHSS_ENABLE_DMA_AHOPPING)
        if (!CC2500_DMA_IS_WORKING(ctx->cc2500)){
            completeChuneChannelDMA(ctx);
            ctx->phase = SFHSS_CONNECTED;
            ctx->stat.proc_endhopping_cnt++;
            ctx->stat.proc_endhopping_time += HRTIMER_GETTIME() - now;
        }
        #endif
        break;
    }
    }

    return rc;
}
