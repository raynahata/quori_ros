#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray, String
from pytz import timezone
from datetime import datetime
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from scipy import signal
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
import matplotlib.pyplot as plt

class ExerciseEval:

    def __init__(self):
        self.sub = rospy.Subscriber("joint_angles", Float64MultiArray, self.callback, queue_size=10)
        self.peaks = []
        self.feedback = []
        self.times = []
        self.pose_detector = mp.solutions.pose.Pose(
                    min_detection_confidence=0.5,  # have some confidence baseline
                    min_tracking_confidence=0.5,
                    model_complexity=0,)
        self.flag = False
        self.threshold1 = 500
        self.threshold2 = 800
        self.threshold3 = 800

    def find_peaks(self,angles):
        grads = np.zeros_like(angles)
        peaks = []
        for joint_ind in range(angles.shape[1]):
            grads[:, joint_ind] = np.gradient(angles[:,joint_ind])
            if joint_ind in self.segmenting_joints:
                peaks.append(signal.find_peaks(grads[:, joint_ind], height=1.5, distance=20, prominence=0.5)[0].astype('int'))
        peaks = np.sort(np.concatenate(peaks))
        # print('Peaks', peaks)
        return peaks, grads

    def check_if_new_peak(self, grads, peak_candidate, index_to_search):
        #Peaks in absolute indices
        #Grads, peak_candidates in relative units

        if (len(self.peaks) == 0 and index_to_search[peak_candidate] > 15) or (len(self.peaks) > 0 and self.peaks[-1] + 15 < index_to_search[peak_candidate]):
            range_to_check = np.arange(np.max([peak_candidate-2, 0]), np.min([peak_candidate+2, grads.shape[0]])).astype('int')

            max_val = np.argmax(grads[range_to_check,:][:,self.segmenting_joints],axis=0)
            max_val = np.mean(max_val).astype('int')

            grad_min = -5
            grad_max = 5

            actual_grad_max = np.max(grads[range_to_check][:, self.segmenting_joints])
            actual_grad_min = np.min(grads[range_to_check][:, self.segmenting_joints])
            if self.exercise_name == 'bicep_curls':
                actual_max = np.max(self.angles[range_to_check][:,self.segmenting_joints[[0, 3]]])
                actual_min = np.min(self.angles[range_to_check][:,self.segmenting_joints[[0, 3]]])
            else:
                actual_max = np.max(self.angles[range_to_check][:,self.segmenting_joints])
                actual_min = np.min(self.angles[range_to_check][:,self.segmenting_joints])

            actual_max_loc = np.where(self.angles[range_to_check][:,self.segmenting_joints] == actual_max)[0][0]
            actual_min_loc = np.where(self.angles[range_to_check][:,self.segmenting_joints] == actual_min)[0][0]
            if (actual_grad_max > grad_max \
                or actual_grad_min < grad_min) \
                    and actual_max_loc > actual_min_loc:
                
                if (self.exercise_name == 'bicep_curls' and actual_min < 50 and actual_max > 100) or self.exercise_name == 'lateral_raises':
                    # print(actual_max_loc, actual_min_loc)
                    # print(range_to_check[max_val], actual_grad_max, actual_grad_min, actual_max, actual_min)
                    # print('-')
                    return range_to_check[max_val]
            
        return False

    def set_joint_groups(self):
        groups = {}
        for joint_ind, joint in enumerate(self.joints):
            if joint[-1] in groups.keys():
                groups[joint[-1]].append(joint_ind)
            else:
                groups[joint[-1]] = [joint_ind]
        self.joint_groups = groups

        joint_to_groups = np.zeros((len(self.joints)))
        counter = 0
        for joint_group, joints in self.joint_groups.items():
            for joint in joints:
                joint_to_groups[joint] = counter
            counter += 1
        
        self.joint_to_groups = np.array(joint_to_groups).astype(int)

    def evaluate_rep(self, current_rep, rep_duration):
        distances = np.zeros((len(self.joints), len(self.experts)))
        for joint_ind in range(len(self.joints)):
            for expert_ind in range(len(self.experts)):
                current_expert = self.experts[expert_ind][:,joint_ind]
                # distance, _ = fastdtw(current_rep[:, joint_ind], current_expert, dist=euclidean)
                distance = 0
                distances[joint_ind, expert_ind] = distance
        
        corrections = []
        for joint_group, joints in self.joint_groups.items():

            #Get the average distance for each expert
            expert_distances = np.mean(distances[joints,:], axis=0)
        
            #Get closest expert
            closest_expert = np.argmin(expert_distances)
            best_distance = np.min(expert_distances)
            expert_label = self.labels[closest_expert]

            if best_distance < self.threshold1:
                correction = expert_label
            elif best_distance < self.threshold2:
                if expert_label == 'Good':
                    correction = 'ok'
                else:
                    correction = 'bad'
            else:
                correction = 'bad'
            
            correction += ' {}'.format(joint_group)
            corrections.append(correction)

        
        if rep_duration < np.mean(self.expert_duration) + 2.5*np.std(self.expert_duration):
            speed = 'fast'
        elif rep_duration > np.mean(self.expert_duration) - 2.5*np.std(self.expert_duration):
            speed = 'slow'
        else:
            speed = 'good'
            
        
        feedback = {'speed': speed, 'correction': corrections}

        return feedback

    def callback(self, data):
        if not self.flag:
            if self.angles.shape[0] > 10 and len(self.peaks) > 0 and self.peaks[-1] + 20 < self.angles.shape[0]:
                #Add last point
                self.peaks.append(self.angles.shape[0]-1)

                #Evaluate rep
                current_rep = self.angles[self.peaks[-2]:self.peaks[-1],:]
                rep_duration = (self.times[self.peaks[-2]] - self.times[self.peaks[-1]]).total_seconds()
                feedback = self.evaluate_rep(current_rep, rep_duration)
                self.feedback.append(feedback)

            return

        #Read angle from message
        angle = data.data
        self.angles = np.vstack((self.angles, angle))

        #Get time
        time = datetime.now(tz)
        self.times.append(time)

        #Look for new peaks
        if self.angles.shape[0] % 10 == 0 and self.angles.shape[0] > 15:
            index_to_search = np.arange(np.max([0,self.angles.shape[0]-500]), self.angles.shape[0]).astype('int')
            
            peak_candidates, grads  = self.find_peaks(self.angles[index_to_search,:])
                
            #Get actual list of peaks
            for peak_candidate in peak_candidates:
                res = self.check_if_new_peak(grads, peak_candidate, index_to_search)
                if res:
                    self.peaks.append(index_to_search[res])
                    print(self.peaks[-1])
                    #Evaluate new rep
                    if len(self.peaks) > 1:
                        current_rep = self.angles[self.peaks[-2]:self.peaks[-1],:]
                        rep_duration = (self.times[self.peaks[-1]] - self.times[self.peaks[-2]]).total_seconds()
                        feedback = self.evaluate_rep(current_rep, rep_duration)
                        self.feedback.append(feedback)
                        print(feedback)

    def plot_results(self):
        print('Peaks', self.peaks)

        fig, ax = plt.subplots(self.angles.shape[1], sharex=True, sharey=True)

        for ii in range(self.angles.shape[1]):
            ax[ii].plot(self.angles[:,ii], 'k', linestyle=':')
            ax[ii].plot(np.gradient(self.angles[:,ii]), 'k')

            counter = 0
            for start, end in zip(self.peaks[:-1], self.peaks[1:]):
                if counter % 2 == 0:
                    style = '--'
                else:
                    style = '-'
                # print(self.feedback, self.joint_to_groups, counter)
                correction = self.feedback[counter]['correction'][self.joint_to_groups[ii]]
                if 'Good' in correction:
                    color = 'g'
                elif 'ok' in correction:
                    color = 'y'
                elif 'bad' in correction:
                    color = 'r'
                elif 'low' in correction:
                    color = 'm'
                elif 'high' in correction:
                    color = 'b'
                else:
                    color = 'k' 

                ax[ii].plot(np.arange(start, end), self.angles[start:end,ii], linestyle=style, color=color)
                

                counter += 1

            ax[ii].set_title('{}-{}-{}-{}'.format(self.joints[ii][0], self.joints[ii][1], self.joints[ii][2], self.joints[ii][3]))
        plt.show()

if __name__ == '__main__':
    
    rospy.init_node('exercise_session', anonymous=True)
    
    #Initialize the publishers/subscribers
    quori_body_face_pub = rospy.Publisher("quori_body_face", Float64MultiArray, queue_size=2)
    quori_sound_pub = rospy.Publisher('quori_sound', String, queue_size=2)
    
    #Initialize exercise evaluation variables
    tz = timezone('EST')

    #Start with exercise 1, set 1
    exercise_eval = ExerciseEval()

    # exercise_eval.exercise_name = 'bicep_curls'
    exercise_eval.exercise_name = 'lateral_raises'
    
    npzfile = np.load('/home/quori4/quori_files/quori_ros/src/quori_exercises/experts/{}_experts.npz'.format(exercise_eval.exercise_name), allow_pickle=True)
    
    exercise_eval.joints, exercise_eval.experts, exercise_eval.expert_duration, exercise_eval.segmenting_joints, exercise_eval.labels = npzfile['joints'], npzfile['experts'], npzfile['expert_duration'], npzfile['segmenting_joints'], npzfile['labels']
    
    exercise_eval.angles = np.empty((0,len(exercise_eval.joints)))

    #Get joint groups
    exercise_eval.set_joint_groups()
    
    rospy.sleep(5)

    inittime = datetime.now(tz)
    
    print('Recording!')
    while (datetime.now(tz) - inittime).total_seconds() < 45:        
            
        exercise_eval.flag = True
    
    exercise_eval.flag = False
    print('-----------------')
    print('Done with exercise')

    #Get summary statistics
    print('Total Number of Reps {}'.format(len(exercise_eval.peaks)-1))
    for feedback in exercise_eval.feedback:
        print(feedback)
    rospy.sleep(10)
    exercise_eval.plot_results()

    exercise_eval.sub.unregister()

    
    





