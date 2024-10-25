#!/usr/bin/env python

import time

import numpy as np
import matplotlib.pyplot as plt


w = 200
b = 100

xy_train = np.array([
    [1, 300],
    [2, 500],
    #[7, 1893],
    #[3, 777],
    #[5, 1200],
], dtype=np.float64)

x_train = xy_train.T[0]
y_train = xy_train.T[1]
print(f'x_train={x_train}')
print(f'y_train={y_train}')

def compute_model_output(x, w, b):
    return w * x + b
    #m = x.shape[0]
    #f_wb = np.zeros(m)
    #for i in range(m):
    #    f_wb[i] = w * x[i] + b
    #return f_wb


def compute_cost(x, y, w, b):
    f_wb = compute_model_output(x, w, b)
    return np.sum(np.square(f_wb - y)) / 2 / x.shape[0]
    #m = x.shape[0]
    #cost = 0.0
    #for i in range(m):
    #    cost += (f_wb[i] - y[i]) ** 2
    #cost = cost / 2 / m
    #return cost


def compute_gradient(x, y, w, b):
    f_wb = compute_model_output(x, w, b)
    dj_db = (f_wb - y) / x.shape[0]
    dj_dw = dj_db * x
    return np.sum(dj_dw), np.sum(dj_db)
    #dj_dw = 0.0
    #dj_db = 0.0
    #f_wb = compute_model_output(x, w, b)
    #m = x.shape[0]
    #for i in range(m):
    #    diff = f_wb[i] - y[i]
    #    dj_dw += diff * x[i]
    #    dj_db += diff
    #return dj_dw/m, dj_db/m


def gradient_descent(x, y, w_in, b_in, alpha, num_iters, cost_function, gradient_function):
    j_hist = []
    p_hist = []
    w = w_in
    b = b_in
    for i in range(num_iters):
        dj_dw, dj_db = gradient_function(x, y, w, b)
        w = w - alpha * dj_dw
        b = b - alpha * dj_db
        if True or i<1000000:
            j_hist.append(cost_function(x, y, w, b))
            p_hist.append([w,b])
        #if False or i<100 or i%10 == 0:
        #    print(f"GRAD_DESCENT_PROGRESS: i={i} cost={j_hist[-1]} dj_dw={dj_dw} dj_db={dj_db} w={w} b={b}")
        if abs(dj_dw) < 1e-6 and abs(dj_db) < 1e-6:
            print(f"GRAD_DESCENT_PROGRESS: i={i} cost={j_hist[-1]} dj_dw={dj_dw} dj_db={dj_db} w={w} b={b}")
            print("END_EPSILON")
            break
    return w, b


t1 = time.time()
w, b = gradient_descent(x_train, y_train, 0, 0, 0.01, 1000000, compute_cost, compute_gradient)
t2 = time.time()
print(f'TIME={t2-t1}')

print(f'END: w={w} b={b}')

