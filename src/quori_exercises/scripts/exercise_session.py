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
import logging
import sys
import os
import multiprocessing

PARTICIPANT_ID = '1'
ROUND_NUM = 1
ROBOT_NUM = 1
EXERCISE_LENGTH = 30
NUM_SETS = 1

class ExerciseEval:

    def __init__(self):
        self.pose_sub = rospy.Subscriber("joint_angles", Float64MultiArray, self.pose_callback, queue_size=10)
        self.face_sub = rospy.Subscriber("facial_features", Float64MultiArray, self.face_callback, queue_size=10)
        self.peaks = []
        self.feedback = []
        self.times = []
        self.face_times = []
        self.pose_detector = mp.solutions.pose.Pose(
                    min_detection_confidence=0.5,  # have some confidence baseline
                    min_tracking_confidence=0.5,
                    model_complexity=0,)
        self.flag = False
        self.threshold1 = [850, 1200]
        self.threshold2 = [1200, 1700]
        self.threshold3 = 700

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
            
            # logger.info('{} - Closest good expert {}, Closest All expert {}, Closest expert label {}'.format(joint_group, np.min(good_distances), np.min(expert_distances), self.labels[np.argmin(expert_distances)]))
            
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
        logger.info(feedback)
        # if np.mean(eval_list) > 0:
        #     quori_sound_pub.publish("{} speed, {} form".format(speed, 'good'))
        # elif np.mean(eval_list) > -1:
        #     quori_sound_pub.publish("{} speed, {} form".format(speed, 'ok'))
        # else:
        #     quori_sound_pub.publish("{} speed, {} form".format(speed, 'bad'))
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
                self.feedback.append(feedback)
                self.performance = np.vstack((self.performance, self.feedback['evaluation']))
                
            return

        #Read angle from message
        angle = angle_data.data
        self.angles = np.vstack((self.angles, np.array(angle)))

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
                    logger.info('Current peak {}'.format(self.peaks[-1]))

                    #Evaluate new rep
                    if len(self.peaks) > 1:
                        current_rep = self.angles[self.peaks[-2]:self.peaks[-1],:]
                        rep_duration = (self.times[self.peaks[-1]] - self.times[self.peaks[-2]]).total_seconds()
                        feedback = self.evaluate_rep(current_rep, rep_duration)
                        self.feedback.append(feedback)
                        self.performance = np.vstack((self.performance, feedback['evaluation']))
                        logger.info('Feedback: {}'.format(feedback))
                        
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

    def face_callback(self, face_data):
        if self.flag:
            self.facial_features = np.vstack((self.facial_features, face_data.data))

            time = datetime.now(tz)
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

    def react(self):
        #Check if last rep was overall good
        if self.performance.shape[0] == 0:
            last_rep = self.performance
        else:
            last_rep = self.performance[-1,:]

        if np.mean(last_rep) > 0:
            logger.info('////Good last rep - emoting happy')
            body_face_msg = Float64MultiArray()
            body_face_msg.data = [1, 0.5, 1]
            quori_body_face_pub.publish(body_face_msg)
            
            #Robot says
            robot_message = "Great job"
            logger.info('Robot says: {}'.format(robot_message))
            # engine.say(robot_message)
            # engine.runAndWait()


def from_file(filename, re_eval):

    data_file = np.load('src/quori_exercises/saved_data/{}'.format(filename), allow_pickle=True)

    exercise_eval = ExerciseEval()
    
    exercise_eval.exercise_name = data_file['exercise_name']

    npzfile = np.load('src/quori_exercises/experts/{}_experts.npz'.format(exercise_eval.exercise_name), allow_pickle=True)
    
    exercise_eval.joints, exercise_eval.experts, exercise_eval.expert_duration, exercise_eval.segmenting_joints, exercise_eval.labels = npzfile['joints'], npzfile['experts'], npzfile['expert_duration'], npzfile['segmenting_joints'], npzfile['labels']

    if exercise_eval.exercise_name == 'lateral_raises':
        exercise_eval.segmenting_joints = [0] + exercise_eval.segmenting_joints

    exercise_eval.angles = data_file['angles']
    exercise_eval.facial_features = data_file['facial_features']
    exercise_eval.peaks=data_file['peaks']
    exercise_eval.feedback=data_file['feedback']
    exercise_eval.times=data_file['times']
    exercise_eval.face_times=data_file['face_times']
   
    exercise_eval.good_experts = np.array([ii for ii, label in enumerate(exercise_eval.labels) if 'Good' in label]).astype(int)

    #Get joint groups
    exercise_eval.set_joint_groups()

    if re_eval:

        #Clear all the computed values
        exercise_eval.peaks=[]
        exercise_eval.feedback=[]

        #Get joint groups
        exercise_eval.set_joint_groups()
        exercise_eval.performance = np.empty((0, len(exercise_eval.joint_groups)))

        exercise_eval.reeval()

    # print(exercise_eval.feedback())

    exercise_eval.plot_results()

if __name__ == '__main__':
    
    #Initialize ROS node
    rospy.init_node('exercise_session', anonymous=True)
    rate = rospy.Rate(10)

    #Initialize the publishers/subscribers
    quori_body_face_pub = rospy.Publisher("quori_body_face", Float64MultiArray, queue_size=2)
    quori_sound_pub = rospy.Publisher("quori_sound", String, queue_size=10)
    rospy.sleep(4)

    #Initialize the timezone
    tz = timezone('EST')

    #Set flag for live or not
    from_file_flag = True

    #Set flags and variables for reaching from files
    re_eval = True
    SET_NUM = 1
    EXERCISE_NAME = 'bicep_curls'
    data_filename = 'Participant_{}_Round_{}_Robot_{}_Exercise_{}_Set_{}.npz'.format(PARTICIPANT_ID, ROUND_NUM, ROBOT_NUM, EXERCISE_NAME, SET_NUM)

    if from_file_flag:

        #Add logging variables
        logger = logging.getLogger()
        formatter = logging.Formatter('%(asctime)s | %(levelname)s | %(message)s')
        stdout_handler = logging.StreamHandler(sys.stdout)
        stdout_handler.setLevel(logging.DEBUG)
        stdout_handler.setFormatter(formatter)
        logger.addHandler(stdout_handler)

        from_file(data_filename, re_eval)
    else:

        #For each exercise and set
        for EXERCISE_NAME in ['bicep_curls']:
            for SET_NUM in range(1, NUM_SETS+1):
                
                #Start log file
                log_filename = 'Participant_{}_Round_{}_Robot_{}_Exercise_{}_Set_{}.log'.format(PARTICIPANT_ID, ROUND_NUM, ROBOT_NUM, EXERCISE_NAME, SET_NUM)
                logger = logging.getLogger()
                formatter = logging.Formatter('%(asctime)s | %(levelname)s | %(message)s')
                stdout_handler = logging.StreamHandler(sys.stdout)
                stdout_handler.setLevel(logging.DEBUG)
                stdout_handler.setFormatter(formatter)
                file_handler = logging.FileHandler('src/quori_exercises/saved_logs/{}'.format(log_filename))
                file_handler.setFormatter(formatter)
                logger.addHandler(file_handler)
                logger.addHandler(stdout_handler)

                #Starting set
                logger.info('=====================================')
                logger.info('STARTING SET {} OF {}'.format(SET_NUM, EXERCISE_NAME))
                logger.info('=====================================')

                #Robot says starting set
                robot_message = "Get ready for set %s out of %s of %s" % (SET_NUM,
                                                                    NUM_SETS, EXERCISE_NAME.replace("_", " " ))
                logger.info('Robot says: {}'.format(robot_message))
                quori_sound_pub.publish(robot_message)
                rospy.sleep(4)

                exercise_eval = ExerciseEval()

                exercise_eval.exercise_name = EXERCISE_NAME
                
                npzfile = np.load('src/quori_exercises/experts/{}_experts.npz'.format(exercise_eval.exercise_name), allow_pickle=True)
                
                exercise_eval.joints, exercise_eval.experts, exercise_eval.expert_duration, exercise_eval.segmenting_joints, exercise_eval.labels = npzfile['joints'], npzfile['experts'], npzfile['expert_duration'], npzfile['segmenting_joints'], npzfile['labels']

                exercise_eval.good_experts = np.array([ii for ii, label in enumerate(exercise_eval.labels) if 'Good' in label]).astype(int)

                if exercise_eval.exercise_name == 'lateral_raises':
                    exercise_eval.segmenting_joints = [0] + exercise_eval.segmenting_joints

                exercise_eval.angles = np.empty((0,len(exercise_eval.joints)))
                exercise_eval.facial_features = np.empty((0,7))

                #Get joint groups
                exercise_eval.set_joint_groups()
                exercise_eval.performance = np.empty((0, len(exercise_eval.joint_groups)))

                inittime = datetime.now(tz)
                logger.info('-------------------Recording!')
                half_message = False
                start_message = False
                while (datetime.now(tz) - inittime).total_seconds() < EXERCISE_LENGTH:        
                
                    #Robot says starting set
                    if not start_message:
                        robot_message = "Start %s now" % (EXERCISE_NAME.replace("_", " " ))
                        logger.info('Robot says: {}'.format(robot_message))
                        quori_sound_pub.publish(robot_message)
                        start_message = True


                    exercise_eval.flag = True
                    if (datetime.now(tz) - inittime).total_seconds() > EXERCISE_LENGTH/2 and not half_message:
                        robot_message = "Halfway"
                        logger.info('Robot says: {}'.format(robot_message))
                        quori_sound_pub.publish(robot_message)
                        rospy.sleep(2)
                        half_message = True
                
                robot_message = "Rest"
                logger.info('Robot says: {}'.format(robot_message))
                quori_sound_pub.publish(robot_message)
                rospy.sleep(2)

                exercise_eval.flag = False
                logger.info('-------------------Done with exercise')

                #Get summary statistics
                logger.info('Total Number of Reps {}'.format(len(exercise_eval.peaks)-1))

                logger.info('Total Angles {}, Total Facial Features {}'.format(exercise_eval.angles.shape[0], exercise_eval.facial_features.shape))

                exercise_eval.pose_sub.unregister()
                exercise_eval.face_sub.unregister()
                
                data_filename = 'Participant_{}_Round_{}_Robot_{}_Exercise_{}_Set_{}.npz'.format(PARTICIPANT_ID, ROUND_NUM, ROBOT_NUM, EXERCISE_NAME, SET_NUM)
                np.savez('src/quori_exercises/saved_data/{}'.format(data_filename),      
                                        angles=exercise_eval.angles,
                                        facial_features=exercise_eval.facial_features,
                                        peaks=exercise_eval.peaks,
                                        feedback=exercise_eval.feedback,
                                        times=exercise_eval.times,
                                        face_times=exercise_eval.face_times,
                                        exercise_name=EXERCISE_NAME
                                    )
                logger.info('Saved file {}'.format(data_filename))

                logging.shutdown()


    
    





