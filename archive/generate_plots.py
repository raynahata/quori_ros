import os
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

def find_peaks(angles, segmenting_joints):
    grads = np.zeros_like(angles)
    peaks = []
    for joint_ind in range(angles.shape[1]):
        grads[:, joint_ind] = np.gradient(angles[:,joint_ind])
        if joint_ind in segmenting_joints:
            peaks.append(signal.find_peaks(grads[:, joint_ind], height=1.5, distance=20, prominence=0.5)[0].astype('int'))
    peaks = np.sort(np.concatenate(peaks))

    return peaks, grads

def check_if_new_peak(angles, grads, peaks, peak_candidate, segmenting_joints, exercise_name):

    #Peaks in absolute indices
    #Grads, peak_candidates in relative units
    
    if (len(peaks) == 0 and peak_candidate > 15) or (len(peaks) > 0 and peaks[-1] + 15 < peak_candidate):
        range_to_check = np.arange(np.max([peak_candidate-4, 0]), np.min([peak_candidate+4, grads.shape[0]])).astype('int')
        
        max_val = np.argmax(grads[range_to_check,:][:,segmenting_joints],axis=0)
        max_val = np.mean(max_val).astype('int')

        grad_min = -5
        grad_max = 5

        actual_grad_max = np.max(grads[range_to_check][:, segmenting_joints])
        actual_grad_min = np.min(grads[range_to_check][:, segmenting_joints])
        if exercise_name == 'bicep_curls':
            actual_max = np.max(angles[range_to_check][:,segmenting_joints])
            actual_min = np.min(angles[range_to_check][:,segmenting_joints])
        else:
            actual_max = np.max(angles[range_to_check][:,segmenting_joints])
            actual_min = np.min(angles[range_to_check][:,segmenting_joints])

        actual_max_loc = np.where(angles[range_to_check][:,segmenting_joints] == actual_max)[0][0]
        actual_min_loc = np.where(angles[range_to_check][:,segmenting_joints] == actual_min)[0][0]

        # print(index_to_search[range_to_check[max_val]], 'Grad Max:', actual_grad_max, 'Grad Min:', actual_grad_min, 'Max Loc:', actual_max_loc, 'Min Loc:', actual_min_loc, 'Actual Max:', actual_max, 'Actual Min:', actual_min)
        if (actual_grad_max > grad_max \
            or actual_grad_min < grad_min) \
                and actual_max_loc > actual_min_loc:
            
            if (exercise_name == 'bicep_curls' and actual_min < 50 and actual_max > 100) or (exercise_name == 'lateral_raises' and actual_max > 100 and actual_min < 50):
                # print('added')
                return range_to_check[max_val]
        
    return False


filename = 'src/quori_exercises/experts/bicep_curls_demos.npz'

npz_file = np.load(filename)

angles = npz_file['angles']

fig, ax = plt.subplots(int(angles.shape[1]/2), 2,  sharex = True, sharey  = True)
counter = [0, 0]
for ii in range(angles.shape[1]):
    if ii % 2 == 0:
        current_column = 0
    else:
        current_column = 1

    ax[counter[current_column], current_column].plot(angles[:300, ii])
    ax[counter[current_column], current_column].set_title('{}-{}-{}-{}'.format(npz_file['joints'][ii][0], npz_file['joints'][ii][1], npz_file['joints'][ii][2], npz_file['joints'][ii][3]))
    counter[current_column] += 1


#Plot gradients
fig, ax = plt.subplots(2, 2, sharex=True, sharey = True)
joints = [4, 5, 8, 9]
counter = [0, 0]
for ii in joints:
    if ii % 2 == 0:
        current_column = 0
    else:
        current_column = 1

    ax[counter[current_column], current_column].plot(angles[:300, ii])
    ax[counter[current_column], current_column].plot(np.gradient(angles[:300, ii]), 'k')
    ax[counter[current_column], current_column].set_title('{}-{}-{}-{}'.format(npz_file['joints'][ii][0], npz_file['joints'][ii][1], npz_file['joints'][ii][2], npz_file['joints'][ii][3]))
    counter[current_column] += 1


#Plot segmentation
#Find peaks
segmenting_joints = [4, 5, 8, 9]
peak_candidates, grads = find_peaks(angles[:300,:], segmenting_joints)

#Get actual list of peaks
peaks = []
for peak_candidate in peak_candidates:
    res = check_if_new_peak(angles[:300,:], grads, peaks, peak_candidate, segmenting_joints, 'bicep_curls')
    if res:
        peaks.append(peak_candidate)

fig, ax = plt.subplots(2, 2, sharex=True, sharey = True)
counter = [0, 0]
for ii in segmenting_joints:
    if ii % 2 == 0:
        current_column = 0
    else:
        current_column = 1

    # ax[counter[current_column], current_column].plot(angles[:300, ii], '--')
    # ax[counter[current_column], current_column].plot(np.gradient(angles[:300, ii]), 'k')
    p_counter = 0
    for start, end in zip(peaks[:-1], peaks[1:]):
        if p_counter % 2 == 0:
            style = '--'
            color = 'g'
        else:
            style = '-'
            color = 'b'

        ax[counter[current_column], current_column].plot(np.arange(start, end), angles[start:end,ii], linestyle=style, color=color)
        p_counter += 1

    ax[counter[current_column], current_column].plot(angles[:300, ii], 'k', linestyle=':')
    ax[counter[current_column], current_column].set_title('{}-{}-{}-{}'.format(npz_file['joints'][ii][0], npz_file['joints'][ii][1], npz_file['joints'][ii][2], npz_file['joints'][ii][3]))
    counter[current_column] += 1


plt.show()