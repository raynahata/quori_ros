#!/usr/bin/env python3
import rospy
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from config import *
import datetime
import pickle

#Storage
all_exercises = {'bicep_curls': {'good': [], 'low range': [], 'high range': []}, 'lateral_raises': {'good': [], 'low range': [], 'high range': []}}

#Bag File Names
bag_names = ['Reid_bicep_curls.bag']

#Labels
labels = [
    ['good', 'good', 'good', 'good', 'low range', 'low range', 'low range', 'good', 'good', 'good', 'good', 'good', 'good', 'good', 'good']
]

#Exercise Names
current_exercises = ['bicep_curls']


for bag_name, current_exercise, label in zip(bag_names, current_exercises, labels):
    bag = rosbag.Bag(bag_name)

    angles = np.empty((0, 12))
    peaks = [0]
    times = []

    for idx, (topic, msg, t) in enumerate(bag.read_messages(topics=['/joint_angles'])):

        if (bag_name == 'Reid_bicep_curls.bag' and idx > 200 and idx < 1820+200):

            #Read angle from message
            angle = msg.data
            angles = np.vstack((angles, np.array(angle)))

            #Get time
            dt = datetime.datetime.utcfromtimestamp(t.to_sec())
            if angles.shape[0] == 1:
                initial_time = dt
            times.append((dt - initial_time).total_seconds())

            #Look for new peaks
            if angles.shape[0] > 70:
            
                #If far enough away from previous peak
                if len(peaks) == 0 or (peaks[-1] + 70 < angles.shape[0]):

                    #Check if new rep

                    if (current_exercise == 'bicep_curls' and np.max(angles[-20:,:][:,EXERCISE_INFO[current_exercise]['segmenting_joint_inds']]) < 40) or \
                    (current_exercise == 'lateral_raises' and np.max(angles[-20:,:][:,EXERCISE_INFO[current_exercise]['segmenting_joint_inds']]) < 35) :

                        #Add peak
                        peaks.append(angles.shape[0]-1)
    
    peaks.append(angles.shape[0] - 1)
    
    times = np.array(times)

    for peak_num, (beg, end) in enumerate(zip(peaks[:-1], peaks[1:])):
        if end-beg > 2:
            time_data = (times[beg:end] - times[beg]).reshape(-1,1)
            angle_data = angles[beg:end,:]
            data = np.hstack([time_data, angle_data])
            all_exercises[current_exercise][label[peak_num]].append(data)

    fig, ax = plt.subplots(4, 3)
    ii = 0
    for row in range(4):
        for col in range(3):
            ax[row, col].plot(angles[:,ii])
            peak_num = 0
            for peak_num, (beg, end) in enumerate(zip(peaks[:-1], peaks[1:])):
                if peak_num % 2 == 0:
                    color = 'b'
                else:
                    color = 'g'
                ax[row, col].plot(np.arange(beg, end), angles[beg:end,ii], color)
                
            ii += 1

    #Save data
    with open('new_experts.pickle', 'wb') as handle:
        pickle.dump(all_exercises, handle, protocol=pickle.HIGHEST_PROTOCOL)

plt.show()