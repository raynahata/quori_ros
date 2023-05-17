#!/usr/bin/env python3
import numpy as np
from scipy import signal
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
import matplotlib.pyplot as plt
import multiprocessing
import rospy
from std_msgs.msg import Float64MultiArray, String
from datetime import datetime
from pytz import timezone
import time

class ExerciseEval:

    def __init__(self, replay, exercise_name, feedback_controller):
        self.replay = replay
        self.flag = False

        if not self.replay:
            #Initialize the subscribers
            self.pose_sub = rospy.Subscriber("joint_angles", Float64MultiArray, self.pose_callback, queue_size=10)
            self.face_sub = rospy.Subscriber("facial_features", Float64MultiArray, self.face_callback, queue_size=10)

        self.peaks = []
        self.feedback = []
        self.times = []
        self.face_times = []
        self.threshold1 = [850, 1200]
        self.threshold2 = [1200, 1700]
        self.feedback_controller = feedback_controller

        #Set the variables from the file
        self.exercise_name = exercise_name
        npzfile = np.load('src/quori_exercises/experts/{}_experts.npz'.format(self.exercise_name), allow_pickle=True)
        self.joints, self.experts, self.expert_duration, self.segmenting_joints, self.labels = npzfile['joints'], npzfile['experts'], npzfile['expert_duration'], npzfile['segmenting_joints'], npzfile['labels']
        self.good_experts = np.array([ii for ii, label in enumerate(self.labels) if 'Good' in label]).astype(int)
        if self.exercise_name == 'lateral_raises':
            self.segmenting_joints = [0] + self.segmenting_joints
        
        #Initialize the angles and facial feature storage
        self.angles = np.empty((0,len(self.joints)))
        self.facial_features = np.empty((0,7))

        #Get joint groups
        self.set_joint_groups()
        self.performance = np.empty((0, len(self.joint_groups)))

    def find_peaks(self,angles):
        grads = np.zeros_like(angles)
        peaks = []
        for joint_ind in range(angles.shape[1]):
            grads[:, joint_ind] = np.gradient(angles[:,joint_ind])
            if joint_ind in self.segmenting_joints:
                peaks.append(signal.find_peaks(grads[:, joint_ind], height=1.5, distance=20, prominence=0.5)[0].astype('int'))
        peaks = np.sort(np.concatenate(peaks))
        return peaks, grads

    def check_if_new_peak(self, grads, peak_candidate, index_to_search):
        #Peaks in absolute indices
        #Grads, peak_candidates in relative units
        
        if (len(self.peaks) == 0 and index_to_search[peak_candidate] > 15) or (len(self.peaks) > 0 and self.peaks[-1] + 15 < index_to_search[peak_candidate]):
            range_to_check = np.arange(np.max([peak_candidate-4, 0]), np.min([peak_candidate+4, grads.shape[0]])).astype('int')
            
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

            # print(index_to_search[range_to_check[max_val]], 'Grad Max:', actual_grad_max, 'Grad Min:', actual_grad_min, 'Max Loc:', actual_max_loc, 'Min Loc:', actual_min_loc, 'Actual Max:', actual_max, 'Actual Min:', actual_min)
            if (actual_grad_max > grad_max \
                or actual_grad_min < grad_min) \
                    and actual_max_loc > actual_min_loc:
                
                if (self.exercise_name == 'bicep_curls' and actual_min < 50 and actual_max > 100) or (self.exercise_name == 'lateral_raises' and actual_max > 100 and actual_min < 50):
                    # print('added')
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

    def calc_dist_worker(self, ind, series1, series2, q):
            q.put((ind, fastdtw(series1, series2, dist=euclidean)))

    def calc_dist(self, current_rep, joints):

        input_values = [self.experts[ii][:,joints] for ii in range(len(self.experts))]
        qout = multiprocessing.Queue()
        processes = [multiprocessing.Process(target=self.calc_dist_worker, args=(ind, current_rep, val, qout))
                for ind, val in enumerate(input_values)]
        
        for p in processes:
            p.start()

        for p in processes:
            p.join()

        unsorted_result = [qout.get() for p in processes]
        result = [t[1][0] for t in sorted(unsorted_result)]

        return result

    def evaluate_rep(self, current_rep, rep_duration):

        corrections = []
        eval_list = []

        for joint_group, joints in self.joint_groups.items():
            
            #Get expert distances per group
            expert_distances = self.calc_dist(current_rep[:, joints], joints)

        
            #Get closest good expert
            if self.exercise_name == 'bicep_curls':
                threshold1 = self.threshold1[0]
                threshold2 = self.threshold2[0]
            else:
                threshold1 = self.threshold1[1]
                threshold2 = self.threshold2[1]
            good_distances = [expert_distances[ii] for ii in self.good_experts]

            if np.min(good_distances) < threshold1:
                closest_expert = self.good_experts[np.argmin(good_distances)]
                best_distance = np.min(good_distances)
            else:
                closest_expert = np.argmin(expert_distances)
                best_distance = np.min(expert_distances)
            expert_label = self.labels[closest_expert]
                        
            if best_distance < threshold1:
                if expert_label == 'Good':
                    eval_list.append(1)
                else:
                    eval_list.append(-1)
                correction = expert_label
            elif best_distance < threshold2:
                if expert_label == 'Good':
                    correction = 'ok'
                    eval_list.append(0)
                else:
                    correction = 'bad'
                    eval_list.append(-1)
            else:
                correction = 'bad'
                eval_list.append(-1)
            
            correction += ' {}'.format(joint_group)
            corrections.append(correction)
        
        if rep_duration < np.mean(self.expert_duration) - 2:
            speed = 'fast'
        elif rep_duration > np.mean(self.expert_duration) + 2:
            speed = 'slow'
        else:
            speed = 'good'
            
        feedback = {'speed': speed, 'correction': corrections, 'evaluation': eval_list}

        self.feedback.append(feedback)
        self.performance = np.vstack((self.performance, feedback['evaluation']))
        
        self.feedback_controller.logger.info(feedback)
        self.feedback_controller.react(self.feedback, self.exercise_name)

        return feedback

    def pose_callback(self, angle_data):

        if not self.flag:
            if self.angles.shape[0] > 10 and len(self.peaks) > 0 and self.peaks[-1] + 20 < self.angles.shape[0]:
                #Add last point
                self.peaks.append(self.angles.shape[0]-1)

                #Evaluate rep
                current_rep = self.angles[self.peaks[-2]:self.peaks[-1],:]
                rep_duration = (self.times[self.peaks[-2]] - self.times[self.peaks[-1]]).total_seconds()
                feedback = self.evaluate_rep(current_rep, rep_duration)

            return

        #Read angle from message
        angle = angle_data.data
        self.angles = np.vstack((self.angles, np.array(angle)))

        #Get time
        time = datetime.now(timezone('EST'))
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
                    self.feedback_controller.logger.info('Current peak {}'.format(self.peaks[-1]))

                    #Evaluate new rep
                    if len(self.peaks) > 1:
                        current_rep = self.angles[self.peaks[-2]:self.peaks[-1],:]
                        rep_duration = (self.times[self.peaks[-1]] - self.times[self.peaks[-2]]).total_seconds()
                        feedback = self.evaluate_rep(current_rep, rep_duration)
                        
                        
    def reeval(self):
        for index, angle in enumerate(self.angles):
            #Look for new peaks
            if index > 15:
                index_to_search = np.arange(0, index).astype('int')
                
                peak_candidates, grads  = self.find_peaks(self.angles[index_to_search,:])
  
                #Get actual list of peaks
                for peak_candidate in peak_candidates:
                    res = self.check_if_new_peak(grads, peak_candidate, index_to_search)
                    if res:
                        self.peaks.append(index_to_search[res])

                        #Evaluate new rep
                        if len(self.peaks) > 1:
                            current_rep = self.angles[self.peaks[-2]:self.peaks[-1],:]
                            rep_duration = (self.times[self.peaks[-1]] - self.times[self.peaks[-2]]).total_seconds()
                            feedback = self.evaluate_rep(current_rep, rep_duration)
                            self.feedback.append(feedback)
                            self.performance = np.vstack((self.performance, feedback['evaluation']))
            time.sleep(0.1)

    def face_callback(self, face_data):
        if self.flag:
            self.facial_features = np.vstack((self.facial_features, face_data.data))

            time = datetime.now(timezone('EST'))
            self.face_times.append(time)

    def plot_results(self):

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
                if counter < len(self.feedback):
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

            ax[ii].set_title('{}-{}-{}-{}-{}'.format(ii,self.joints[ii][0], self.joints[ii][1], self.joints[ii][2], self.joints[ii][3]), fontsize = 8)
        plt.show()

