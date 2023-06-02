#!/usr/bin/env python3
import pyttsx3
from datetime import datetime
import syllables
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
import numpy as np

# from gtts import gTTS
# from time import sleep
# import os
# import pyglet

# tts = gTTS(text='Get ready for bicep curls now', lang='en')
# filename = '/tmp/temp.mp3'
# tts.save(filename)

# music = pyglet.media.load(filename, streaming=False)
# music.play()

# sleep(music.duration) #prevent from killing
# os.remove(filename) #remove temperory file

from gtts import gTTS
import pygame
from io import BytesIO

pygame.init()

def say(text):
    tts = gTTS(text=text, lang='en')
    fp = BytesIO()
    tts.write_to_fp(fp)
    fp.seek(0)
    pygame.mixer.init()
    pygame.mixer.music.load(fp)
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy():
        pygame.time.Clock().tick(10)



intercept = 0.6477586140350873
slope = 0.31077594

m = 'Get ready for bicep curls now'
start = datetime.now()
say(m)
end = datetime.now()

print('Actual', (end - start).total_seconds())
print('Estimated', slope*syllables.estimate(m)+intercept)

# engine = pyttsx3.init()
# voices = engine.getProperty('voices')

# rate = 150
# engine.setProperty('rate', rate)

# message = 'Starting bicep curls now'
# syllable_estimate = syllables.estimate(message)
# time_estimate = syllable_estimate*slope + intercept
# print('Estimate', np.round(time_estimate))

# for id in range(len(voices)):
#     print(voices[id].name, voices[id].languages, voices[id].age, voices[id].gender)
#     if voices[id].name in ['default']:
#         engine.setProperty('voice', voices[id].id)
#         start = datetime.now()
#         engine.say(message)
#         engine.runAndWait()
#         end = datetime.now()
#         length = (end-start).total_seconds()
#         print('Time', np.round(length))

# time_arrs = []

# for m in range(1, 20):
#     message = 't'*m

#     start = datetime.now()
#     say(message)
#     end = datetime.now()

#     length = (end-start).total_seconds()
#     time_arrs.append(length)
#     print('Num Syllables', m, 'Time', length)

# print(time_arrs)

x = np.arange(1, 20).reshape((-1, 1))
y = np.array([1.001924, 1.4256, 1.532458, 1.938388, 2.243875, 2.45061, 2.861637, 2.965487, 3.26843, 3.583136, 3.894372, 4.500796, 4.802016, 5.11022, 5.52887, 5.624075, 5.934113, 6.141939, 6.546897])
model = LinearRegression()
model.fit(x, y)
print(f"intercept: {model.intercept_}")
print(f"slope: {model.coef_}")
y_pred = model.predict(x)
plt.plot(x, y)
plt.plot(x, y_pred)
plt.plot()
plt.show()