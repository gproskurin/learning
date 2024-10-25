#!/usr/bin/env python

import copy
import time

import numpy as np
import matplotlib.pyplot as plt

X_train = np.array([
    [2104, 5, 1, 45],
    [1416, 3, 2, 40],
    [852, 2, 1, 35]


    #[1, 300],
    #[2, 500],
    #[7, 1893],
    #[3, 777],
    #[5, 1200],
], dtype=np.float64)

N_FEATURES = X_train.shape[1]
w_init = np.zeros(N_FEATURES)
b_init = 0

y_train = np.array([460, 232, 178])
print(f'X_train={X_train}')
print(f'y_train={y_train}')

def compute_model_output(x, w, b):
    return np.dot(w, x) + b
    #m = x.shape[0]
    #f_wb = np.zeros(m)
    #for i in range(m):
    #    f_wb[i] = w * x[i] + b
    #return f_wb


#def compute_cost(x, y, w, b):
#    f_wb = compute_model_output(x, w, b)
#    return np.sum(np.square(f_wb - y)) / 2 / x.shape[0]
#    #m = x.shape[0]
#    #cost = 0.0
#    #for i in range(m):
#    #    cost += (f_wb[i] - y[i]) ** 2
#    #cost = cost / 2 / m
#    #return cost

def compute_cost(X, y, w, b):
    (m_samples, n_features) = X.shape
    cost = 0
    for i in range(m_samples):
        f_wb_i = compute_model_output(X[i], w, b)
        cost += (f_wb_i - y[i]) ** 2
    cost = cost / (2 * m_samples)
    return cost


#def compute_gradient(x, y, w, b):
#    f_wb = compute_model_output(x, w, b)
#    dj_db = (f_wb - y) / x.shape[0]
#    dj_dw = dj_db * x
#    return np.sum(dj_dw), np.sum(dj_db)
#    #dj_dw = 0.0
#    #dj_db = 0.0
#    #f_wb = compute_model_output(x, w, b)
#    #m = x.shape[0]
#    #for i in range(m):
#    #    diff = f_wb[i] - y[i]
#    #    dj_dw += diff * x[i]
#    #    dj_db += diff
#    #return dj_dw/m, dj_db/m

def compute_gradient(X, y, w, b):
    (m_samples, n_features) = X.shape
    dj_dw = np.zeros(n_features)
    dj_db = 0
    for i_sample in range(m_samples):
        err = compute_model_output(X[i_sample], w, b) - y[i_sample]

        for j_feature in range(n_features):
            dj_dw[j_feature] += err * X[i_sample][j_feature]

        dj_db += err

    dj_db = dj_db / m_samples
    dj_dw = dj_dw / m_samples
    return dj_dw, dj_db


EPS = 1e-6
def is_small(x):
    return abs(x) < EPS

def is_small_array(a):
    for x in a:
        if not is_small(x):
            return False
    return True

def gradient_descent(X, y, w_in, b_in, alpha, num_iters, cost_function, gradient_function):
    j_hist = []
    p_hist = []
    w = copy.deepcopy(w_in)
    b = b_in
    for i in range(num_iters):
        dj_dw, dj_db = gradient_function(X, y, w, b)
        w = w - alpha * dj_dw
        b = b - alpha * dj_db
        if True or i<1000000:
            j_hist.append(cost_function(X, y, w, b))
            p_hist.append([w,b])
        #if False or i<100 or i%10 == 0:
        #    print(f"GRAD_DESCENT_PROGRESS: i={i} cost={j_hist[-1]} dj_dw={dj_dw} dj_db={dj_db} w={w} b={b}")
        if is_small_array(dj_dw) and is_small(dj_db):
            print(f"GRAD_DESCENT_PROGRESS: i={i} cost={j_hist[-1]} dj_dw={dj_dw} dj_db={dj_db} w={w} b={b}")
            print("END_EPSILON")
            break
    return w, b



t1 = time.time()
w, b = gradient_descent(
    X=X_train,
    y=y_train,
    w_in=w_init,
    b_in=b_init,
    alpha=5e-7,
    num_iters=1000,
    cost_function=compute_cost,
    gradient_function=compute_gradient
)
t2 = time.time()
print(f'TIME={t2-t1}')

print(f'END: w={w} b={b}')

for i_sample in range(X_train.shape[0]):
    p = compute_model_output(X_train[i_sample], w, b)
    print(f'sample[{i_sample}]: prediction={p} value={y_train[i_sample]}')

