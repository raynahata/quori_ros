#!/usr/bin/env python3
import pyttsx3
from datetime import datetime
import syllables
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
import numpy as np

intercept = 0.6121186140350887
slope = 0.19308147

engine = pyttsx3.init()
voices = engine.getProperty('voices')

rate = 150
engine.setProperty('rate', rate)

message = 'Starting bicep curls now'
syllable_estimate = syllables.estimate(message)
time_estimate = syllable_estimate*slope + intercept
print('Estimate', np.round(time_estimate))

for id in range(len(voices)):
    engine.setProperty('voice', voices[id].id)
    start = datetime.now()
    engine.say(message)
    engine.runAndWait()
    end = datetime.now()
    length = (end-start).total_seconds()
    print('Time', np.round(length))

# time_arrs = []

# for m in range(1, 20):
#     message = 't'*m

#     start = datetime.now()
#     engine.say(message)
#     engine.runAndWait()
#     end = datetime.now()

#     length = (end-start).total_seconds()
#     time_arrs.append(length)
#     print('Num Syllables', m, 'Time', length)

# print(time_arrs)

# x = np.arange(1, 20).reshape((-1, 1))
# y = np.array([0.818842, 1.019779, 1.146254, 1.391905, 1.524725, 1.773641, 2.023303, 2.142914, 2.39705, 2.516248, 2.767083, 2.895945, 3.146031, 3.274838, 3.523963, 3.642148, 3.895265, 4.147735, 4.268064])
# model = LinearRegression()
# model.fit(x, y)
# print(f"intercept: {model.intercept_}")
# print(f"slope: {model.coef_}")
# y_pred = model.predict(x)
# plt.plot(x, y)
# plt.plot(x, y_pred)
# plt.plot()
# plt.show()