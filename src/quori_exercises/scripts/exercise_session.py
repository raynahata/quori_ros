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

class CameraSubscriber:

    def __init__(self):
        self.sub = rospy.Subscriber("/astra_ros/devices/default/color/image_color", Image, self.callback)
        self.all_peaks = [0]
        self.all_feedback = []
        self.all_landmarks = []
        self.all_times = []
        self.landmark_points = ['nose', 'left_eye_inner', 'left_eye', 'left_eye_outer', 'right_eye_inner', 'right_eye', 'right_eye_outer', 'left_ear', 'right_ear', 'mouth_left', 'mouth_right', 'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow', 'left_wrist', 'right_wrist', 'left_pinky', 'right_pinky', 'left_index', 'right_index', 'left_thumb', 'right_thumb', 'left_hip', 'right_hip', 'left_knee', 'right_knee', 'left_ankle', 'right_ankle', 'left_heel', 'right_heel', 'left_foot_index', 'right_foot_index']
        self.pose_detector = mp.solutions.pose.Pose(
                    min_detection_confidence=0.5,  # have some confidence baseline
                    min_tracking_confidence=0.5,
                    model_complexity=0,)
        self.flag = False
        self.threshold1 = 1000
        self.threshold2 = 1500
        self.threshold3 = 800
    
    def calc_angle(self, vec_0, vec_1, angle_type):
        if angle_type == 'xy':
            angle = np.arctan2(vec_1[1], vec_1[1]) - \
                np.arctan2(-vec_0[1], vec_0[0])
        elif angle_type == 'yz':
            angle = np.arctan2(vec_1[1], vec_1[2]) - \
                np.arctan2(-vec_0[1], -vec_0[2])
        elif angle_type == 'xz':
            angle = np.arctan2(vec_1[2], vec_1[0]) - \
                np.arctan2(-vec_0[2], -vec_0[0])
        
        angle = np.abs(angle*180.0/np.pi)
        if angle > 180:
            angle = 360-angle

        return 180 - angle

    def find_peaks(self, angles):
        grads = []
        peaks = []
        for joint_ind, joint in enumerate(self.npzfile['joints']):
            grads.append(np.gradient(angles[:,joint_ind]))
            if int(joint[4]) == -1: #Only consider segmenting joints
                peaks.append(signal.find_peaks(grads[joint_ind], height=1.5, distance=20, prominence=0.5)[0].astype('int'))
            if int(self.npzfile['joints'][joint_ind][4]) == -2: #Only consider segmenting joints
                peaks.append(signal.find_peaks(angles[:,joint_ind], height=50, distance=20, prominence=1.0)[0].astype('int'))
        peaks = np.sort(np.concatenate(peaks))
        grads = np.array(grads).T
        # print('Peaks', peaks)
        return peaks, grads

    def check_if_new_peak(self, angles, grads, peak_candidate, index_to_search):

        #Peaks in absolute indices
        #angles, Grads, peak_candidates in relative units
        if (len(self.all_peaks) == 0 and index_to_search[peak_candidate] > 20) or (len(self.all_peaks) > 0 and self.all_peaks[-1] + 30 < index_to_search[peak_candidate]):
            
            range_to_check = np.arange(np.max([peak_candidate-5, 0]), np.min([peak_candidate+5, angles.shape[0]])).astype('int')
            max_val = np.argmax(angles[range_to_check,:][:,self.segmenting_joints],axis=0)
            
            max_val = np.mean(max_val).astype('int')
            
            if self.exercise_name == 'bicep_curls':
                grad_min = -3
                grad_max = 3
            else:
                grad_min = -3
                grad_max = 3

            if (np.max(grads[range_to_check][:,self.segmenting_joints]) > grad_max \
                    or np.min(grads[range_to_check][:,self.segmenting_joints]) > grad_min)  or \
                (len(self.all_peaks) > 0 \
                    and np.abs(self.all_peaks[-1] - index_to_search[range_to_check[max_val]]) > 50 \
                    and np.max(grads[range_to_check][:,self.segmenting_joints]) > 0.8 \
                    and np.min(grads[range_to_check][:,self.segmenting_joints]) > -0.8):
                return range_to_check[max_val]
        
        return False

    def evaluate_rep(self, angles, start_time, end_time):

        all_distances = np.zeros((len(self.npzfile['joints']), len(self.npzfile['experts'])))
        for joint_ind, joint in enumerate(self.npzfile['joints']):
            for ii in range(len(self.npzfile['experts'])):
                current_expert = self.npzfile['experts'][ii][:,joint_ind]
                distance, path = fastdtw(angles[:,joint_ind], current_expert, dist=euclidean)
                all_distances[joint_ind,ii] = distance
            
        #Get closest expert for each joint
        closest_expert = np.argmin(all_distances, axis=1)
        best_distances = [all_distances[ii, e] for ii, e in enumerate(closest_expert)]
        expert_messages = [self.npzfile['messages'][e] for e in closest_expert]
        
        forms = []
        for joint_ind, (d, m) in enumerate(zip(best_distances, expert_messages)):
            if d < self.threshold1 and m == 'good':
                form = 'good'
            elif d < self.threshold2 and m == 'good':
                form = 'ok'
            elif d < self.threshold3 and not (m == 'good'):
                form = m
            else:
                form = 'bad'
            forms.append(form)
        
        #Get the mode for each joint_group
        all_f = []
        for joint_name, group in self.npzfile['joint_groups'].item().items():
            values_for_group = []
            low_count = 0
            high_count = 0
            for g in group:
                val = forms[g]
                if val == 'good':
                    values_for_group.append(1.0)
                elif val == 'ok':
                    values_for_group.append(0.0)
                else:
                    if 'low' in val:
                        low_count += 1
                    elif 'too' in val:
                        high_count += 1
                    values_for_group.append(-1.0)
            avg_group = np.mean(values_for_group)

            if avg_group < -0.5:
                #If more than one range of motion comments
                
                if low_count > 1:
                    f = 'Low range of motion for {}'.format(joint_name)

                elif high_count > 1: 
                    f = 'Too much range of motion for {}'.format(joint_name)
                
                else: 
                    f = 'Bad form for {}'.format(joint_name)
            
            elif avg_group > 0.3:
                f = 'Good form {}'.format(joint_name)
            
            else:
                f = 'Ok form for {}'.format(joint_name)
            all_f.append(f)

        #duration 
        start = datetime.strptime(start_time, "%m/%d/%Y/%H:%M:%S")
        end = datetime.strptime(end_time, "%m/%d/%Y/%H:%M:%S")
        length = (end - start).total_seconds()
        
        if length > np.mean(self.npzfile['lengths']) + 2*np.std(self.npzfile['lengths']):
            speed = 'fast'
        elif length < np.mean(self.npzfile['lengths']) - 2*np.std(self.npzfile['lengths']):
            speed = 'slow'
        else:
            speed = 'good'
        
        feedback = {'speed': speed, 'form': forms, 'distances': all_distances, 'closest_expert': closest_expert, 'best_distances': best_distances, 'groups': all_f}

        print('{} speed, {}'.format(feedback['speed'],feedback['groups']))
        return feedback

    def callback(self, data):
        if not self.flag:
            return

        print('Recording!')
        image = np.frombuffer(data.data, dtype=np.uint8).reshape(
            data.height, data.width, -1)
        results = self.pose_detector.process(image)
        
        if results.pose_landmarks:
            ct = datetime.now(tz)
            row = []
            row.append(ct.strftime("%m/%d/%Y/%H:%M:%S"))

            landmarks = {}
            for i, landmark in enumerate(results.pose_landmarks.landmark):
                landmarks[self.landmark_points[i]] = [landmark.x, landmark.y, landmark.z]
                row.append(landmarks[self.landmark_points[i]])
            self.all_landmarks.append(row)

            self.all_times.append(ct.strftime("%m/%d/%Y/%H:%M:%S"))

            angle = np.zeros((len(self.npzfile['joints'])))
            for joint_ind, joint in enumerate(self.npzfile['joints']):
                #Get current angles
                point_0 = np.array(landmarks[joint[0]])
                point_1 = np.array(landmarks[joint[1]])
                point_2 = np.array(landmarks[joint[2]])

                vec_0 = point_0 - point_1
                vec_1 = point_2 - point_1

                angle[joint_ind] = self.calc_angle(vec_0, vec_1, joint[3])
            
            self.all_angles = np.vstack((self.all_angles, angle))

            #Look for peaks in the last 20 points
            if self.all_angles.shape[0] > 20:
                index_to_search = np.arange(np.max([0,self.all_angles.shape[0]-50]), self.all_angles.shape[0]).astype('int')
                
                peak_candidates, grads  = self.find_peaks(self.all_angles[index_to_search,:])
                
                for peak_candidate in peak_candidates:
                    res = self.check_if_new_peak(self.all_angles[index_to_search], grads, peak_candidate, index_to_search)

                    if res:
                        self.all_peaks.append(index_to_search[res])
                        
                        #Evaluate new rep
                        if len(self.all_peaks) > 1:
                            current_rep = self.all_angles[self.all_peaks[-2]:self.all_peaks[-1]]
                            feedback = self.evaluate_rep(current_rep, self.all_times[self.all_peaks[-2]], self.all_times[self.all_peaks[-1]])

                            self.all_feedback.append(feedback)
                            
    def plot_results(self):
        # Evaluation plot
        fig1, ax1 = plt.subplots(len(self.npzfile['joints']) + 1, 1, sharex=True, sharey=True)
        # fig2, ax2 = plt.subplots(1, len(npzfile['joints']))

        for joint_ind, joint in enumerate(self.npzfile['joints']):
            ax1[joint_ind].plot(self.all_angles[:,joint_ind], 'k', linestyle=':')
            
            distances = []
            for rep_ind in range(len(self.all_feedback)):
                if self.all_feedback[rep_ind]['form'][joint_ind] == 'good':
                    color = 'g'
                elif self.all_feedback[rep_ind]['form'][joint_ind] == 'ok':
                    color = 'y'
                elif not self.all_feedback[rep_ind]['form'][joint_ind] == 'bad':
                    color = 'm'
                else:
                    color = 'r'

                mid_point_x = np.mean([self.all_peaks[rep_ind], self.all_peaks[rep_ind+1]]).astype('int')
                mid_point_y = self.all_angles[mid_point_x, joint_ind]

                if joint_ind == 0:
                    text = 'Speed: {}'.format(self.all_feedback[rep_ind]['speed'])
                    for f in self.all_feedback[rep_ind]['groups']:
                        text += '\n {}'.format(f)
                    if rep_ind % 2:
                        ax1[len(self.npzfile['joints'])].annotate(text, xy=(self.all_peaks[rep_ind], 50), xytext=(self.all_peaks[rep_ind], 50), arrowprops=dict(facecolor='black', shrink=0.05))
                    else:
                        ax1[len(self.npzfile['joints'])].annotate(text, xy=(self.all_peaks[rep_ind], 50), xytext=(self.all_peaks[rep_ind], 120), arrowprops=dict(facecolor='black', shrink=0.05))

                if rep_ind % 2:
                    ax1[joint_ind].plot(np.arange(self.all_peaks[rep_ind], self.all_peaks[rep_ind+1]), self.all_angles[self.all_peaks[rep_ind]: self.all_peaks[rep_ind+1], joint_ind], color, linestyle='--', linewidth=4)

                    # ax1[joint_ind].annotate(all_feedback[rep_ind]['speed'], xy=(mid_point_x, mid_point_y), xytext=(mid_point_x, mid_point_y+20), arrowprops=dict(facecolor='black', shrink=0.05))
                else:
                    ax1[joint_ind].plot(np.arange(self.all_peaks[rep_ind], self.all_peaks[rep_ind+1]), self.all_angles[self.all_peaks[rep_ind]: self.all_peaks[rep_ind+1], joint_ind], color, linewidth=4)
                    # ax1[joint_ind].annotate(all_feedback[rep_ind]['speed'], xy=(mid_point_x, mid_point_y), xytext=(mid_point_x, mid_point_y+35), arrowprops=dict(facecolor='black', shrink=0.05))

                ax1[joint_ind].set_title('{} angle'.format(joint[5]))

                distances.append(self.all_feedback[rep_ind]['distances'][joint_ind,:])
            
            distances = np.array(distances).T
            # im = ax2[joint_ind].imshow(distances, cmap='Greys')
            # ax2[joint_ind].set_title('{} angle'.format(joint[5]))

            # demo_labels = ['Demo {}'.format(ii) for ii in range(distances.shape[0])]
            # ax2[joint_ind].set_yticks(np.arange(distances.shape[0]))
            # ax2[joint_ind].set_yticklabels(demo_labels)

            # rep_labels = ['Rep {}'.format(ii) for ii in range(distances.shape[1])]
            # ax2[joint_ind].set_xticks(np.arange(distances.shape[1]))
            # ax2[joint_ind].set_xticklabels(rep_labels)

            # fig2.colorbar(im, ax=ax2[joint_ind])


        plt.show()

if __name__ == '__main__':

    rospy.init_node('exercise_session', anonymous=True)

    #Initialize the publishers/subscribers
    quori_body_face_pub = rospy.Publisher("quori_body_face", Float64MultiArray, queue_size=2)
    quori_sound_pub = rospy.Publisher('quori_sound', String, queue_size=2)
    
    #Initialize exercise evaluation variables
    tz = timezone('EST')

    #Walk through an exercise session

    #Consent first
    quori_sound_pub.publish("When consent is completed, press enter on the keyboard")

    input("Press Enter to continue to training...")

    #Training next
    training_message = "Training information here"

    input("Press Enter to continue to exercises...")

    #Start with exercise 1, set 1
    cam_sub = CameraSubscriber()

    cam_sub.exercise_name = 'bicep_curls'
    cam_sub.npzfile = np.load('/home/quori4/quori_files/quori_ros/src/quori_exercises/experts/{}.npz'.format(cam_sub.exercise_name), allow_pickle=True)

    cam_sub.all_angles = np.empty((0,len(cam_sub.npzfile['joints'])))
    
    cam_sub.segmenting_joints = []
    for joint_ind, joint in enumerate(cam_sub.npzfile['joints']):
        if int(joint[4]) == -1:
            cam_sub.segmenting_joints.append(joint_ind)

    inittime = datetime.now(tz)

    while (datetime.now(tz) - inittime).total_seconds() < 20:        
            
        cam_sub.flag = True
    
    cam_sub.flag = False
    print('Done with exercise')
    cam_sub.plot_results()
    rospy.spin()


    
    





