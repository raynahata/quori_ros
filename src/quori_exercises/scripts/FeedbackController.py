#!/usr/bin/env python3
import logging
import sys
from datetime import datetime
from pytz import timezone
import numpy as np
from std_msgs.msg import Float64MultiArray, String
import rospy
import syllables

class FeedbackController:
    def __init__(self, replay, log_filename, robot_num):
        self.flag = False
        self.replay = replay
        self.logger = logging.getLogger()
        formatter = logging.Formatter('%(asctime)s | %(levelname)s | %(message)s')
        stdout_handler = logging.StreamHandler(sys.stdout)
        stdout_handler.setLevel(logging.DEBUG)
        stdout_handler.setFormatter(formatter)
        self.logger.addHandler(stdout_handler)

        if not self.replay:
            #Initialize the publishers/subscribers
            self.body_face_pub = rospy.Publisher("quori_body_face", Float64MultiArray, queue_size=2)
            self.sound_pub = rospy.Publisher("quori_sound", String, queue_size=10)
        
            #Add file handler
            file_handler = logging.FileHandler('src/quori_exercises/saved_logs/{}'.format(log_filename))
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)
        
        self.message_log = []
        self.message_time_stamps = []
        self.eval_case_log = []
        self.speed_case_log = []
        self.robot_num = int(robot_num)
        self.intercept = 0.6121186140350887
        self.slope = 0.19308147

    def message(self, m, priority=2):
        #Only message if it has been 3 sec since last message ended
        if (len(self.message_time_stamps)) > 0:
            last_message_time = self.message_time_stamps[-1]
            if (datetime.now(timezone('EST')) - last_message_time).total_seconds() < 2.5 and priority < 2:
                #Skip message
                self.logger.info('Skipping {}'.format(m))
                return
                
        self.logger.info('Robot says: {}'.format(m))
        length_estimate = np.round(self.slope*syllables.estimate(m) + self.intercept)
        if not self.replay:
            self.sound_pub.publish(m)
        self.message_log.append(m)
        self.message_time_stamps.append(datetime.now(timezone('EST')) + datetime.timedelta(seconds=length_estimate) )
    
    def find_eval_case(self, feedback):
        c = ''

        #Case 1: 2 bad in a row per joint (only if the last thing said has not been a case 1)
        if len(feedback) >= 2:
            bad_joint_groups = []
            for joint_group in range(len(feedback[-1]['evaluation'])):        

                if 'low range' in feedback[-2]['correction'][joint_group] and 'low range' in feedback[-1]['correction'][joint_group]:
                    bad_joint_groups.append('low range')
                elif 'bad' in feedback[-2]['correction'][joint_group] and 'bad' in feedback[-1]['correction'][joint_group]:
                    bad_joint_groups.append('bad')
                elif 'high range' in feedback[-2]['correction'][joint_group] and 'high range' in feedback[-1]['correction'][joint_group]:
                    bad_joint_groups.append('high range')
                else:
                    bad_joint_groups.append('')
        
            if 'low range' in bad_joint_groups:
                #Both sides or just one side?
                indices =[index for index, item in enumerate(bad_joint_groups) if item == 'low range']

                #Only right side
                if (0 in indices or 2 in indices) and not (1 in indices or 3 in indices):
                   c= '1a'
                
                #Only left side
                elif not (0 in indices or 2 in indices) and (1 in indices or 3 in indices):
                   c= '1b'
                
                else:
                   c= '1c'
            
            elif 'high range' in bad_joint_groups:
                #Both sides or just one side?
                indices =[index for index, item in enumerate(bad_joint_groups) if item == 'high range']

                #Only right side
                if (0 in indices or 2 in indices) and not (1 in indices or 3 in indices):
                   c= '1d'
                
                #Only left side
                elif not (0 in indices or 2 in indices) and (1 in indices or 3 in indices):
                   c= '1e'
                
                else:
                   c= '1f'
            
            elif 'bad' in bad_joint_groups:
                #Both sides or just one side?
                indices =[index for index, item in enumerate(bad_joint_groups) if item == 'bad']

                #Only right side
                if (0 in indices or 2 in indices) and not (1 in indices or 3 in indices):
                   c= '1g'
                
                #Only left side
                elif not (0 in indices or 2 in indices) and (1 in indices or 3 in indices):
                   c= '1h'
                
                else:
                   c= '1i'

            if len(self.eval_case_log) >= 2:
                # print(self.eval_case_log, '1' in self.eval_case_log[-1], '1' in self.eval_case_log[-2])
                if '1' in self.eval_case_log[-1] or '1' in self.eval_case_log[-2]:
                    c = ''

        #Case 2a: 2 bad eval followed by good eval
        if len(feedback) >= 3:
            bad_joints = []
            for joint_group in range(len(feedback[-1]['evaluation'])):
                if feedback[-3]['evaluation'][joint_group] == -1 and feedback[-2]['evaluation'][joint_group] == -1:
                    bad_joints.append(joint_group)
            
            if len(bad_joints) > 0:
                #Check if next one is good or ok
                if np.min(feedback[-1]['evaluation']) >= 0:
                   c = '2a'
        
        #Case 2b: 3 good eval in a row
        if len(feedback) >= 3:
            if np.min(feedback[-1]['evaluation']) >= 0 and np.min(feedback[-2]['evaluation']) >= 0 and np.min(feedback[-3]['evaluation']) >= 0:
                
                c= '2b'

                #last positive message
                if '2b' in self.eval_case_log:
                    # finding the last occurrence
                    final_index = max([index for index, item in enumerate(self.eval_case_log) if item == '2b'])
                    if final_index + 3 >= len(self.eval_case_log):
                       c= ''
        # print(c)
        return c

    def find_speed_case(self, feedback):
        c = ''

        #Case 3: 2 bad in a row per joint
        if len(feedback) >= 2:
            if feedback[-2]['speed'] == 'fast' and feedback[-1]['speed'] == 'fast':
                c = '3a'
            if feedback[-2]['speed'] == 'slow' and feedback[-1]['speed'] == 'slow':
                c = '3b'
                
        
        #Case 4a: 2 bad speed followed by 2 good speed
        if len(feedback) >= 4:

            if not feedback[-4]['speed'] == 'good' and not feedback[-3]['speed'] == 'good' and feedback[-2]['speed'] == 'good' and feedback[-1]['speed'] == 'good':
               c = '4a'
        
        #Case 4b: 4 good speed in row
        if len(feedback) >= 3:
            if feedback[-3]['speed'] == 'good' and feedback[-2]['speed'] == 'good' and feedback[-1]['speed'] == 'good':
                
                c= '4b'

                #last positive message
                if '4b' in self.speed_case_log:
                    # finding the last occurrence
                    final_index = max([index for index, item in enumerate(self.speed_case_log) if item == '4b'])
                    if final_index + 3 >= len(self.speed_case_log):
                       c= ''
        
        return c

    def get_message(self, c, exercise_name):

        if c == '1a': #Right low range of motion
            if exercise_name == 'bicep_curls':
                if self.robot_num == 1:
                    options = ['']
                elif self.robot_num == 2:
                    options = ['Focus on fully extending your right elbow.', 'Extend your right elbow more.']
                elif self.robot_num == 3:
                    options = ['Great work. Can you focus on extending your right elbow?', 'Great job. Can you try extending your right elbow a bit more?']
            else:
                if self.robot_num == 1:
                    options = ['']
                elif self.robot_num == 2:
                    options = ['Make sure you are not moving your right elbow.', 'Make sure your right elbow is straight.']
                elif self.robot_num == 3:
                    options = ['Great job, try to keep your right elbow straight.', 'Nice, can you focus on keeping your right elbow straight?']
        elif c == '1b': #Left low range of motion
            if exercise_name == 'bicep_curls':
                if self.robot_num == 1:
                    options = ['']
                elif self.robot_num == 2:
                    options = ['Focus on fully extending your left elbow.', 'Extend your left elbow more.']
                elif self.robot_num == 3:
                    options = ['Great work. Can you focus on extending your left elbow?', 'Great job. Can you try extending your left elbow a bit more?']
            else:
                if self.robot_num == 1:
                    options = ['']
                elif self.robot_num == 2:
                    options = ['Make sure you are not moving your left elbow.', 'Make sure your left elbow is straight.']
                elif self.robot_num == 3:
                    options = ['Great job, try to keep your left elbow straight.', 'Nice, can you focus on keeping your left elbow straight?']
        elif c == '1c': #Both sides low range of motion
            if exercise_name == 'bicep_curls':
                if self.robot_num == 1:
                    options = ['']
                elif self.robot_num == 2:
                    options = ['Focus on fully extending your elbows.', 'Extend your elbows more.']
                elif self.robot_num == 3:
                    options = ['Great work. Can you focus on extending your elbows?', 'Great job. Can you try extending your elbows a bit more?']
            else:
                if self.robot_num == 1:
                    options = ['']
                elif self.robot_num == 2:
                    options = ['Make sure you are not moving your elbows.', 'Make sure your elbows are straight.']
                elif self.robot_num == 3:
                    options = ['Great job, try to keep your elbows straight.', 'Nice, can you focus on keeping your elbows straight?']
        elif c == '1d': #Right high range of motion
                if self.robot_num == 1:
                    options = ['']
                elif self.robot_num == 2:
                    options = ['Focus on stopping your right arm at 90 degrees.', 'Make sure your right arm is stopping at 90 degrees.']
                elif self.robot_num == 3:
                    options = ['Great work, can you focus on stopping your right arm at 90 degrees?', 'Nice job, can you make sure you are stopping your right arm at 90 degrees']
        elif c == '1e': #Left high range of motion
                if self.robot_num == 1:
                    options = ['']
                elif self.robot_num == 2:
                    options = ['Focus on stopping your left arm at 90 degrees.', 'Make sure your left arm is stopping at 90 degrees.']
                elif self.robot_num == 3:
                    options = ['Great work, can you focus on stopping your right arm at 90 degrees?', 'Nice job, can you make sure you are stopping your left arm at 90 degrees?']
        elif c == '1f': #Both sides high range of motion
                if self.robot_num == 1:
                    options = ['']
                elif self.robot_num == 2:
                    options = ['Focus on stopping your arms at 90 degrees.', 'Make sure your arms is stopping at 90 degrees.']
                elif self.robot_num == 3:
                    options = ['Great work, can you focus on stopping your arms at 90 degrees?', 'Nice job, can you make sure you are stopping your arms at 90 degrees?']
        elif c == '1g': #Right bad
            if exercise_name == 'bicep_curls':
                if self.robot_num == 1:
                    options = ['']
                elif self.robot_num == 2:
                    options = ['Focus on your right elbow.', 'Pay more attention to your right elbow.']
                elif self.robot_num == 3:
                    options = ['You are doing great, can you focus a bit more on your right elbow?', 'You got this, can you focus a bit more on your right elbow?']
            else:
                if self.robot_num == 1:
                    options = ['']
                elif self.robot_num == 2:
                    options = ['Focus on getting a full range of motion on your right shoulder.', 'Make sure your right arm is getting a full range of motion.']
                elif self.robot_num == 3:
                    options = ['Great work, try to focus a bit more on your right shoulder.', 'Nice job, try to focus on your right shoulder a little more']
        elif c == '1h': #Left bad
            if exercise_name == 'bicep_curls':
                if self.robot_num == 1:
                    options = ['']
                elif self.robot_num == 2:
                    options = ['Focus on your left elbow.', 'Pay more attention to your left elbow.']
                elif self.robot_num == 3:
                    options = ['You are doing great, can you focus a bit more on your left elbow?', 'You got this, can you focus a bit more on your left elbow?']
            else:
                if self.robot_num == 1:
                    options = ['']
                elif self.robot_num == 2:
                    options = ['Focus on getting a full range of motion on your left shoulder.', 'Make sure your left arm is getting a full range of motion.']
                elif self.robot_num == 3:
                    options = ['Great work, try to focus a bit more on your left shoulder.', 'Nice job, try to focus on your left shoulder a little more.']
        elif c == '1i': #generic bad
            if exercise_name == 'bicep_curls':
                if self.robot_num == 1:
                    options = ['']
                elif self.robot_num == 2:
                    options = ['Focus on your elbows.', 'Pay more attention to your elbows.']
                elif self.robot_num == 3:
                    options = ['You are doing great, can you focus a bit more on your elbows?', 'You got this, can you focus a bit more on your elbows?']
            else:
                if self.robot_num == 1:
                    options = ['']
                elif self.robot_num == 2:
                    options = ['Focus on getting a full range of motion in your shoulders.', 'Make sure your arms are getting a full range of motion.']
                elif self.robot_num == 3:
                    options = ['Great work, try to focus a bit more on your shoulders.', 'Nice job, try to focus on your shoulders a little more.']
        elif c == '2a': #2 bad followed by good
            if self.robot_num == 1:
                options = ['']
            elif self.robot_num == 2:
                options = ['Form is good, keep it up.', 'Form looks good, keep going.']
            elif self.robot_num == 3:
                options = ['Looks good, great job!', 'Great work, looking good!']
        elif c == '2b': #4 good - but only every 3 or so
            if self.robot_num == 1:
                options = ['']
            elif self.robot_num == 2:
                options = ['Form is good, keep it up.', 'Form looks good, keep going.']
            elif self.robot_num == 3:
                options = ['Looks good, great job!', 'Great work, looking good!']
        elif c == '3a': #2 fast in a row
            if self.robot_num == 1:
                options = ['']
            elif self.robot_num == 2:
                options = ['Try to slow down.', 'Make sure you do not go too fast.']
            elif self.robot_num == 3:
                options = ['Nice job, can you slow down a little on the next few?', 'Great work, can you try to slow down on the next few?']
        elif c == '3b': #2 slow in a row
            if self.robot_num == 1:
                options = ['']
            elif self.robot_num == 2:
                options = ['Try to speed up.', 'Make sure you do not go too slow.']
            elif self.robot_num == 3:
                options = ['Nice job, can you speed up a little on the next few?', 'Great work, can you try to speed up on the next few?']
        elif c == '4a': #2 bad followed by 1 good speed
            if self.robot_num == 1:
                options = ['']
            elif self.robot_num == 2:
                options = ['Great speed, keep it up.', 'Nice speed, keep going.']
            elif self.robot_num == 3:
                options = ['Great work, nice pace.', 'Nice job, great speed.']
        elif c == '4b': #4 good speed, but only every 3 or so
            if self.robot_num == 1:
                options = ['']
            elif self.robot_num == 2:
                options = ['Great speed, keep it up.', 'Nice speed, keep going.']
            elif self.robot_num == 3:
                options = ['Great work, nice pace.', 'Nice job, great speed.']
        else:
            options = ['']

        if len(options) > 0:
            #Pick the option that has been chosen the least
            counts = []
            for option in options:
                if option in self.message_log:
                    counts.append(self.message_log.count(option))
                else:
                    counts.append(0)
            
            #Get the minimum count
            ind = np.argmin(counts)
            
            return options[ind]
        
        return ''

    def react(self, feedback, exercise_name):

        #Nonverbal
        
        #If last rep was overall good and good speed
        if np.min(feedback[-1]['evaluation']) >= 0  and feedback[-1]['speed'] == 'good':
            self.logger.info('Good evaluation and speed, robot smiles')
            if not self.replay:
                body_face_msg = Float64MultiArray()
                body_face_msg.data = [1, 0.7, 2]
                self.body_face_pub.publish(body_face_msg)
                
        #If last rep was overall good but not good speed
        elif np.min(feedback[-1]['evaluation']) >= 0:
            self.logger.info('Good evaluation, but not good speed, robot smiles')
            if not self.replay:
                body_face_msg = Float64MultiArray()
                body_face_msg.data = [1, 0.5, 2]
                self.body_face_pub.publish(body_face_msg)
        
        elif feedback[-1]['speed'] == 'good':
            self.logger.info('Good speed, but not good evaluation, robot smiles')
            if not self.replay:
                body_face_msg = Float64MultiArray()
                body_face_msg.data = [1, 0.5, 2]
                self.body_face_pub.publish(body_face_msg)
        
        else:
            self.logger.info('Bad evaluation and speed, robot neutral')
            
        
        # Verbal
        # self.message('Rep')
        eval_case = self.find_eval_case(feedback)
        self.eval_case_log.append(eval_case)

        speed_case = self.find_speed_case(feedback)
        self.speed_case_log.append(speed_case)

        #Get message for each case
        eval_message = self.get_message(eval_case, exercise_name)
        speed_message = self.get_message(speed_case, exercise_name)
        
        self.logger.info('Evaluation case {} with message - {}'.format(eval_case, eval_message))
        self.logger.info('Speed case {} with message - {}'.format(speed_case, speed_message))
        
        
        #If both messages available, choose the eval message
        if speed_message == '' and not eval_message == '':
            self.message(eval_message, priority=1)
        elif not speed_message == '' and eval_message == '':
            self.message(speed_message, priority=1)
        elif not speed_message == '' and not eval_message == '':
            self.message(eval_message, priority=1)
            
