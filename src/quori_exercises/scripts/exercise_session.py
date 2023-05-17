#!/usr/bin/env python3
import rospy
from pytz import timezone
from datetime import datetime
import numpy as np

import sys
import os

from ExerciseEval import ExerciseEval
from FeedbackController import FeedbackController

PARTICIPANT_ID = '2'
ROUND_NUM = 1
ROBOT_NUM = 1
EXERCISE_LENGTH = 30
NUM_SETS = 1


def replay(filename, re_eval):

    #Initialize the feedback controller
    feedback_controller = FeedbackController(True, '', ROBOT_NUM)

    data_file = np.load('src/quori_exercises/saved_data/{}'.format(filename), allow_pickle=True)

    exercise_eval = ExerciseEval(True, data_file['exercise_name'], feedback_controller)

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

    exercise_eval.plot_results()

def live_session(exercise_name, set_num):
    #Start log file
    log_filename = 'Participant_{}_Round_{}_Robot_{}_Exercise_{}_Set_{}.log'.format(PARTICIPANT_ID, ROUND_NUM, ROBOT_NUM, EXERCISE_NAME, SET_NUM)

    #Initialize the feedback controller
    feedback_controller = FeedbackController(False, log_filename, ROBOT_NUM)

    #Initialize evaluation object
    exercise_eval = ExerciseEval(False, EXERCISE_NAME, feedback_controller)

    #Starting set
    exercise_eval.feedback_controller.logger.info('=====================================')
    exercise_eval.feedback_controller.logger.info('STARTING SET {} OF {}'.format(SET_NUM, EXERCISE_NAME))
    exercise_eval.feedback_controller.logger.info('=====================================')

    #Robot says starting set
    robot_message = "Get ready for set %s out of %s of %s" % (SET_NUM,
                                                        NUM_SETS, EXERCISE_NAME.replace("_", " " ))
    exercise_eval.feedback_controller.message(robot_message)

    inittime = datetime.now(timezone('EST'))
    exercise_eval.feedback_controller.logger.info('-------------------Recording!')
    half_message = False
    start_message = False
    while (datetime.now(timezone('EST')) - inittime).total_seconds() < EXERCISE_LENGTH:        
    
        #Robot says starting set
        if not start_message:
            robot_message = "Start %s now" % (EXERCISE_NAME.replace("_", " " ))
            exercise_eval.feedback_controller.message(robot_message)
            start_message = True

        exercise_eval.flag = True
        feedback_controller.flag = True
        if (datetime.now(timezone('EST')) - inittime).total_seconds() > EXERCISE_LENGTH/2 and not half_message:
            robot_message = "Halfway"
            exercise_eval.feedback_controller.message(robot_message)
            half_message = True
    
    exercise_eval.flag = False
    exercise_eval.feedback_controller.logger.info('-------------------Done with exercise')

    robot_message = "Rest"
    exercise_eval.feedback_controller.message(robot_message)

    #Get summary statistics
    exercise_eval.feedback_controller.logger.info('Total Number of Reps {}'.format(len(exercise_eval.peaks)-1))

    exercise_eval.feedback_controller.logger.info('Total Angles {}, Total Facial Features {}'.format(exercise_eval.angles.shape[0], exercise_eval.facial_features.shape))

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
    exercise_eval.feedback_controller.logger.info('Saved file {}'.format(data_filename))

    exercise_eval.feedback_controller.logger.shutdown()


if __name__ == '__main__':
    
    #Initialize ROS node
    rospy.init_node('exercise_session', anonymous=True)

    #Set flag for live or not
    replay_flag = True

    if replay_flag:
        #Set flags and variables for reaching from files
        re_eval = True
        SET_NUM = 1
        EXERCISE_NAME = 'bicep_curls'

        data_filename = 'Participant_{}_Round_{}_Robot_{}_Exercise_{}_Set_{}.npz'.format(PARTICIPANT_ID, ROUND_NUM, ROBOT_NUM, EXERCISE_NAME, SET_NUM)

        replay(data_filename, re_eval)
    else:

        rate = rospy.Rate(10)

        #For each exercise and set
        for EXERCISE_NAME in ['bicep_curls']:
            for SET_NUM in range(1, NUM_SETS+1):
                live_session(EXERCISE_NAME, SET_NUM)


               

    
    





