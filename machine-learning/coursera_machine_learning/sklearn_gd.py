#!/usr/bin/env python3

import numpy as np
import sklearn.preprocessing
import sklearn.linear_model


import data


scaler = sklearn.preprocessing.StandardScaler()
regressor = sklearn.linear_model.SGDRegressor(max_iter=1000)

X_train = np.array(data.X_train_lab)
y_train = np.array(data.y_train_lab)
X_norm = scaler.fit_transform(X_train)

regressor.fit(X_norm, y_train)

x_train_test = scaler.transform(X_train, copy=True)
y_test = regressor.predict(x_train_test)

for i in range(y_test.shape[0]):
    print(f"sample[{i}]: prediction={y_test[i]} value={y_train[i]}")


print("\nLOGISTIC REGRESSION\n")
X = np.array([[0.5, 1.5], [1,1], [1.5, 0.5], [3, 0.5], [2, 2], [1, 2.5]])
y = np.array([0, 0, 0, 1, 1, 1])

lr_model = sklearn.linear_model.LogisticRegression()
lr_model.fit(X, y)
y_predict = lr_model.predict(X)

print(f"LR: X={X} y={y} y_predict={y_predict} score={lr_model.score(X,y)}")

