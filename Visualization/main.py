import time
import subprocess
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re
from threading import Thread


def animate(i, x_data, y_data, z_data, fps_text, last_time, max_x):
    x_values = list(x_data.get_data()[1])
    y_values = list(y_data.get_data()[1])
    z_values = list(z_data.get_data()[1])

    if len(x_values) < max_x:
        x_values += [i[0]]
        y_values += [i[1]]
        z_values += [i[2]]
    else:
        x_values = x_values[1:] + [i[0]]
        y_values = y_values[1:] + [i[1]]
        z_values = z_values[1:] + [i[2]]

    x_data.set_data(range(len(x_values)), x_values)
    y_data.set_data(range(len(y_values)), y_values)
    z_data.set_data(range(len(z_values)), z_values)

    new_time = time.time()
    fps_text.set_text("{0:.2f} fps".format(1. / (new_time - last_time[0])))
    last_time.update({0: new_time})


def enqueue_output(out, lines):
    for line in iter(out.readline, b''):
        lines.append(line)

    out.close()


def open_gyro(lines):
    args = ["D:\\Documents\\Helicoptere RC\\Segger RTT over ST-Link\\strtt.exe", "-ram", "4"]
    p = subprocess.Popen(args, stdout=subprocess.PIPE, bufsize=1, text=True)
    Thread(target=enqueue_output, args=(p.stdout, lines), daemon=True).start()
    return p


def read_gyro(lines):
    regex = re.compile(r".*GYRO: +(-?\d*) +(-?\d*) +(-?\d*)\n")
    while True:
        if lines:
            line = lines[-1]
            result = re.findall(regex, line)
            if result:
                yield int(result[0][0]), int(result[0][1]), int(result[0][2])


def main():
    # Setting up the plot
    fig = plt.figure("Raw Sensor Data")
    ax1 = fig.add_subplot(1, 1, 1)
    x_data, = ax1.plot([0], [0], 'r', label='X - roll')
    y_data, = ax1.plot([0], [0], 'g', label='Y - pitch')
    z_data, = ax1.plot([0], [0], 'b', label='Z - yaw')
    fps_text = ax1.text(0.97, 0.03, "", transform=ax1.transAxes, ha="right", va="top")
    ax1.legend()

    plt.ylim(-32768, 32767)

    MAX_X = 1000
    plt.xlim(0, MAX_X)

    LINES = deque(maxlen=1)
    LAST_TIME = {0: time.time()}

    proc = open_gyro(LINES)

    ani = FuncAnimation(fig,
                        animate,
                        read_gyro(LINES),
                        fargs=(x_data, y_data, z_data, fps_text, LAST_TIME, MAX_X),
                        interval=0)

    plt.show()

    proc.wait()


if __name__ == '__main__':
    main()
    