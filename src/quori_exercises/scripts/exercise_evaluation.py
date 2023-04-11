#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import numpy as np
import mediapipe as mp
from scipy import signal
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
import matplotlib.pyplot as plt

def calc_angle(vec_0, vec_1):
    angle = np.arctan2(vec_1[1], vec_1[1]) - np.arctan2(-vec_0[1], vec_0[0])
    angle = np.abs(angle*180.0/np.pi)
    if angle > 180:
        angle = 360-angle

    return 180 - angle


def get_angle(joint, landmarks):
    point_0 = np.array(landmarks[joint[0]])
    point_1 = np.array(landmarks[joint[1]])
    point_2 = np.array(landmarks[joint[2]])
    vec_0 = point_0 - point_1
    vec_1 = point_2 - point_1
    angle = calc_angle(vec_0, vec_1)

    return angle

def evaluate_rep(angles, experts):
    distances = []
    for ii in range(experts.shape[0]):
        current_expert = experts[ii]
        distance, path = fastdtw(angles, current_expert, dist=euclidean)
        distances.append(distance)
    min_dist = np.min(distances)
    closest_expert = np.argmin(distances)
    # if min_dist < 700:
    #     print('Correct execution, close to expert demonstration', closest_expert)
    # elif min_dist < 1000:
    #     print('Execution could be better, somewhat close to expert demonstration, closest expert', closest_expert)
    # else:
    #     print('Incorrect execution, far from all experts')
    return distances


def callback(data):
    image = np.frombuffer(data.data, dtype=np.uint8).reshape(
        data.height, data.width, -1)
    results = POSE_DETECTOR.process(image)
    if results.pose_landmarks:
        landmarks = {}
        for i, landmark in enumerate(results.pose_landmarks.landmark):
            landmarks[POINT_NAMES[i]] = [landmark.x, landmark.y, landmark.z]

        for joint_ind, joint in enumerate(JOINTS_OF_INTEREST):
            angle = get_angle(JOINTS_OF_INTEREST[joint_ind], landmarks)
            ALL_ANGLES[joint_ind].append(angle)

            # Check if in a new rep
            if len(ALL_ANGLES[joint_ind]) > 20:
                    grad = np.gradient(ALL_ANGLES[joint_ind])
                    test_peaks = signal.find_peaks(grad, height=2, distance=30, prominence=0.5)[0].astype('int')
                
                    if len(test_peaks) > 0:
                        PEAK_CANDIDATES[joint_ind].append(test_peaks[-1])

                    if len(PEAK_CANDIDATES[joint_ind]) > 0:
                        current_peak_candidate = PEAK_CANDIDATES[joint_ind][-1]
                        counts = np.count_nonzero(PEAK_CANDIDATES[joint_ind][-5:] == current_peak_candidate)
                    
                        if counts == 5 and (len(PEAKS[joint_ind]) == 0 or PEAKS[joint_ind][-1]+10 < current_peak_candidate):
                            
                            #Hooray it's a new rep
                            range_to_check = np.arange(current_peak_candidate-8, np.min([current_peak_candidate+8, len(ALL_ANGLES[joint_ind])-1])).astype('int')
                            max_val = np.argmax([ALL_ANGLES[joint_ind][ii] for ii in range_to_check])
                            
                            if EXERCISE_NAME == 'bicep_curls':
                                if np.min(grad[range_to_check[max_val]-5:range_to_check[max_val]+5]) > -10:
                                    
                                    PEAKS[joint_ind].append(range_to_check[max_val])

                                    #We've found the beginning of a rep
                                    if len(PEAKS[joint_ind]) > 1:
                                        #Now evaluate it!
                                        current_rep = ALL_ANGLES[joint_ind][PEAKS[joint_ind][-2]:PEAKS[joint_ind][-1]]
                                        ALL_ANGLES_SEGMENTS[joint_ind].append(current_rep)
                                        ALL_SEGMENT_INDS[joint_ind].append(np.arange(PEAKS[joint_ind][-2],PEAKS[joint_ind][-1]))
                                        ALL_DISTANCES[joint_ind].append(evaluate_rep(current_rep, NPZFILE['experts'][joint_ind, :]))
                            else:
                                PEAKS[joint_ind].append(range_to_check[max_val])

                                #We've found the beginning of a rep
                                if len(PEAKS[joint_ind]) > 1:
                                    #Now evaluate it!
                                    current_rep = ALL_ANGLES[joint_ind][PEAKS[joint_ind][-2]:PEAKS[joint_ind][-1]]
                                    ALL_ANGLES_SEGMENTS[joint_ind].append(current_rep)
                                    ALL_SEGMENT_INDS[joint_ind].append(np.arange(PEAKS[joint_ind][-2],PEAKS[joint_ind][-1]))
                                    ALL_DISTANCES[joint_ind].append(evaluate_rep(current_rep, NPZFILE['experts'][joint_ind, :]))
        

    
    #Plot!
    print(len(ALL_ANGLES[0]))
    if len(ALL_ANGLES[0]) > 500:
        fig1, ax1 = plt.subplots(2, 1, sharex=True, sharey=True)
        fig2, ax2 = plt.subplots(1, 2)
        for joint_ind, joint in enumerate(JOINTS_OF_INTEREST):
            ax1[joint_ind].plot(ALL_ANGLES[joint_ind], 'k', linestyle=':')

            angle_name = '{}-{}-{}'.format(joint[0], joint[1], joint[2])
            segment_counter = 0
            for segment_inds, segment, distances in zip(ALL_SEGMENT_INDS[joint_ind], ALL_ANGLES_SEGMENTS[joint_ind], ALL_DISTANCES[joint_ind]):
                if np.min(distances) < THRESHOLD1:
                    color = 'g'
                elif np.min(distances) < THRESHOLD2:
                    color = 'y'
                else:
                    color = 'r'
                if segment_counter % 2:
                    ax1[joint_ind].plot(segment_inds, segment, color, linestyle = '--', linewidth=4)
                else:
                    ax1[joint_ind].plot(segment_inds, segment, color, linewidth=4)
                segment_counter += 1
            
            ax1[joint_ind].set_title('Angle {}'.format(angle_name))

            #Distances plot
            im = ax2[joint_ind].imshow(ALL_DISTANCES[joint_ind], cmap='Greys')
            ax2[joint_ind].set_title('Angle {}'.format(angle_name))

            demo_labels = ['Demo {}'.format(ii) for ii in range(len(NPZFILE['experts'][joint_ind]))]
            ax2[joint_ind].set_xticks(np.arange(len(NPZFILE['experts'][joint_ind])))
            ax2[joint_ind].set_xticklabels(demo_labels)

            rep_labels = ['Rep {}'.format(ii) for ii in range(len(ALL_ANGLES_SEGMENTS[joint_ind]))]
            ax2[joint_ind].set_yticks(np.arange(len(ALL_ANGLES_SEGMENTS[joint_ind])))
            ax2[joint_ind].set_yticklabels(rep_labels)

            fig2.colorbar(im, ax=ax2[joint_ind])

            for ii in range(len(rep_labels)):
                #Get the index of the minimum
                closest = np.argmin(ALL_DISTANCES[joint_ind][ii])
                min_dist = np.min(ALL_DISTANCES[joint_ind][ii])
                if min_dist < THRESHOLD1:
                    color = 'g'
                elif min_dist < THRESHOLD2:
                    color='y'
                else:
                    color = 'r'
                text = ax2[joint_ind].text(closest, ii, np.round(min_dist), ha="center", va="center", color=color, fontsize=12)

        plt.show()
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('exercise_evaluation', anonymous=True)

    rospy.Subscriber(
        "/astra_ros/devices/default/color/image_color", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    POINT_NAMES = [
    'nose',
    'left_eye_inner',
    'left_eye',
    'left_eye_outer',
    'right_eye_inner',
    'right_eye',
    'right_eye_outer',
    'left_ear',
    'right_ear',
    'mouth_left',
    'mouth_right',
    'left_shoulder',
    'right_shoulder',
    'left_elbow',
    'right_elbow',
    'left_wrist',
    'right_wrist',
    'left_pinky',
    'right_pinky',
    'left_index',
    'right_index',
    'left_thumb',
    'right_thumb',
    'left_hip',
    'right_hip',
    'left_knee',
    'right_knee',
    'left_ankle',
    'right_ankle',
    'left_heel',
    'right_heel',
    'left_foot_index',
    'right_foot_index',
    ]

    JOINTS_OF_INTEREST = [['right_shoulder', 'right_elbow', 'right_wrist'], [
        'left_shoulder', 'left_elbow', 'left_wrist']]
    EXERCISE_NAME = 'bicep_curls'
    NPZFILE = np.load('/home/quori4/quori_files/quori_ros/src/quori_exercises/experts/{}.npz'.format(EXERCISE_NAME), allow_pickle=True)

    ALL_ANGLES = []
    ALL_ANGLES_SEGMENTS = []
    ALL_SEGMENT_INDS = []
    ALL_DISTANCES = []
    PEAK_CANDIDATES = []
    PEAKS = []

    for joint in JOINTS_OF_INTEREST:
        ALL_ANGLES.append([])
        ALL_ANGLES_SEGMENTS.append([])
        ALL_SEGMENT_INDS.append([])
        ALL_DISTANCES.append([])
        PEAK_CANDIDATES.append([])
        PEAKS.append([])

    POSE_DETECTOR = mp.solutions.pose.Pose(
        min_detection_confidence=0.5,  # have some confidence baseline
        min_tracking_confidence=0.5,
        model_complexity=0,)


    THRESHOLD1 = 600
    THRESHOLD2 = 1200

    INDEX = 0

    listener()
