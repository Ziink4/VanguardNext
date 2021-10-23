import time
import subprocess
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re
from threading import Thread
from queue import Empty, LifoQueue

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)
data,  = ax1.plot([0], [0])
text = ax1.text(0.97, 0.97, "", transform=ax1.transAxes, ha="right", va="top")
plt.ylim(-32768, 32767)

MAX_X = 1000
plt.xlim(0, MAX_X)

y_values = []
p, q, t = None, None, None
last_time = {0: time.time()}


def animate(i):
    global y_values
    if len(y_values) < MAX_X:
        y_values.append(i)
    else:
        y_values = y_values[1:] + [i]

    data.set_data(range(len(y_values)), y_values)

    new_time = time.time()
    text.set_text("{0:.2f} fps".format(1. / (new_time - last_time[0])))
    last_time.update({0:new_time})


def enqueue_output(out, queue):
    for line in iter(out.readline, b''):
        queue.put(line)
    out.close()


def open_gyro():
    args = ["D:\\Documents\\Helicoptere RC\\Segger RTT over ST-Link\\strtt.exe", "-ram", "4"]
    global p, q, t
    p = subprocess.Popen(args, stdout=subprocess.PIPE, bufsize=1, text=True)
    q = LifoQueue()
    t = Thread(target=enqueue_output, args=(p.stdout, q))
    t.daemon = True
    t.start()


def read_gyro():
    global q
    while True:
        try:
            line = q.get_nowait()
        except Empty:
            pass
        else:
            print(q.qsize())
            result = re.findall(r"\x1b\[0m\x1b\[32mINFO: GYRO: +(-?\d*)\n", line)
            if result:
                yield int(result[0])


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    open_gyro()

    ani = FuncAnimation(fig, animate, read_gyro, interval=0)

    plt.show()

    p.wait()
