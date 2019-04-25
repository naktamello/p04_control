# coding=utf-8
import json
import matplotlib.pyplot as plt
import numpy as np

DELTA = 0.1


def time_in_sec(time_from_start):
    return time_from_start['secs'] + time_from_start['nsecs'] / 1e9


def fit_cubic_spline(n, x, dt, vi, vf):
    g = [0.0] * n
    r = [0.0] * n
    x1 = [0.0] * n
    x2 = [0.0] * n
    # forward sweep of TDMA
    g[0] = 0.5
    r[0] = 3.0 * ((x[1] - x[0]) / dt[0] - vi) / dt[0]
    for i in range(1, n - 1):
        dt2 = dt[i - 1] + dt[i]
        a = dt[i - 1] / dt2
        denom = 2.0 - a * g[i - 1]  # b1 - a1*g0
        g[i] = (1.0 - a) / denom
        r[i] = 6.0 * ((x[i + 1] - x[i]) / dt[i] - (x[i] - x[i - 1]) / dt[i - 1]) / dt2
        r[i] = (r[i] - a * r[i - 1]) / denom
    denom = dt[n - 2] * (2.0 - g[n - 2])
    r[n - 1] = 6.0 * (vf - (x[n - 1] - x[n - 2]) / dt[n - 2])
    r[n - 1] = (r[n - 1] - dt[n - 2] * r[n - 2]) / denom
    # backsubstitution step
    x2[n - 1] = r[n - 1]
    for i in range(n - 2, -1, -1):
        x2[i] = r[i] - g[i] * x2[i + 1]

    x1[0] = vi
    for i in range(1, n - 1):
        x1[i] = (x[i + 1] - x[i]) / dt[i] - (2 * x2[i] + x2[i + 1]) * dt[i] / 6.0
    x1[n - 1] = vf
    return x1, x2


def format_data(points):
    x = []
    v = []
    a = []
    dt = []
    t = []
    prev_t = 0
    for point in points:
        x.append(point['positions'][0])
        v.append(point['velocities'][0])
        a.append(point['accelerations'][0])
        s = time_in_sec(point['time_from_start'])
        t.append(s)
        dt.append(s - prev_t)
        prev_t = s
    return x, v, a, dt, t


class Spline:
    def __init__(self, tim1, tim0, acc1, acc0, pos1, pos0):
        self.tim1 = tim1
        self.tim0 = tim0
        self.acc1 = acc1
        self.acc0 = acc0
        self.pos1 = pos1
        self.pos0 = pos0
        self.h = tim1 - tim0

    def __call__(self, val, velocity=False):
        if velocity:
            return self.velocity(val)
        return self.spline(val)

    def spline(self, val):
        return (self.tim1 - val) ** 3 * self.acc0 / (6 * self.h) + \
               (val - self.tim0) ** 3 * self.acc1 / (6 * self.h) + \
               (self.pos0 / self.h - self.acc0 * self.h / 6) * (self.tim1 - val) + \
               (self.pos1 / self.h - self.acc1 * self.h / 6) * (val - self.tim0)

    def velocity(self, val):
        return -(self.tim1 - val)**2 * self.acc0 / (2*self.h) + \
               (val - self.tim0)**2 * self.acc1 / (2*self.h) + \
               ((self.pos1-self.pos0)/self.h - self.h*(self.acc1-self.acc0)/6)

def make_splines(acc, tim, pos):
    for i in range(1, len(x2)):
        h = tim[i] - tim[i - 1]
        # def make_spline(val):
        #     return (tim[i] - val) ** 3 * acc[i - 1] / (6 * h) + (val - tim[i - 1]) ** 3 * acc[i] / (6 * h) + \
        #            (pos[i - 1] / h - acc[i - 1] * h / 6) * (tim[i] - val) + (pos[i]/h - acc[i]*h/6)*(val-tim[i-1])
        yield Spline(tim[i], tim[i-1], acc[i], acc[i-1], pos[i], pos[i-1])

if __name__ == '__main__':
    with open("topics.json") as infile:
        data = json.loads(infile.read())
    points = data['arm'][0]['goal']['trajectory']['points']
    x, v, a, dt, t = format_data(points)
    x1, x2 = fit_cubic_spline(len(x), x, dt[1:], v[0], v[-1])
    splines = []
    for spline in make_splines(x2, t, x):
        splines.append(spline)

    plt.plot(t, x, marker='o', linestyle='None', color='r', label='pos')
    # for i in range(len(x)):
    #     time_slices = np.linspace(t[i] - DELTA, t[i] + DELTA, 10)
    #     def y(xi):
    #         return v[i] * (xi - t[i]) + x[i] + 0.1
    #     slope = [y(j) for j in time_slices]
    #     plt.plot(time_slices, slope, linestyle='solid', color='b', label='p{}_v'.format(i))
    for i in range(len(splines)):
        time_slices = np.linspace(t[i], t[i+1], 10)
        slope = [splines[i](j) for j in time_slices]
        plt.plot(time_slices, slope, linestyle='solid', color='g' if i%2 else 'b', label='p{}_v1'.format(i))
        print(t[i+1], splines[i](t[i+1], velocity=True), x1[i+1])
    plt.xlim([-.2, max(t) + 0.2])
    plt.xlabel('Time(s)')
    plt.ylabel('Position(rad)')
    plt.title('Cubic Spline')
    # plt.legend()
    plt.show()

    # print x1, x2
