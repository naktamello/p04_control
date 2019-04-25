# coding=utf-8
import json


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
    prev_t = 0
    for point in points:
        x.append(point['positions'][0])
        v.append(point['velocities'][0])
        a.append(point['accelerations'][0])
        s = time_in_sec(point['time_from_start'])
        dt.append(s - prev_t)
        prev_t = s
    return x, v, a, dt


if __name__ == '__main__':
    with open("topics.json") as infile:
        data = json.loads(infile.read())
    points = data['arm'][0]['goal']['trajectory']['points']
    x, v, a, dt = format_data(points)
    x1, x2 = fit_cubic_spline(len(x), x, dt[1:], v[0], v[-1])
    print x1, x2
