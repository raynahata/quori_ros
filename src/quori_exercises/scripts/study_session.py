#!/usr/bin/env python3
import rospy
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from config import *
from feedback_controller import FeedbackController
from exercise_evaluation import ExerciseEval
from datetime import datetime
from pytz import timezone
import logging
import time

#Parameters
MIN_LENGTH = 100
MAX_LENGTH = 150
MAX_REPS = 40
NUM_SETS = 1
REST_TIME = 40
ROUND_REST_TIME = 80
NUM_ROUNDS = 1 
EXERCISE_LIST = ['bicep_curls']

#Change at beginning of study
PARTICIPANT_ID = '1'
VERBAL_CADENCE = 2 #1 is low, 2 is medium, 3 is high
NONVERBAL_CADENCE = 2
ROBOT_STYLE = 3

#Initialize ROS node
rospy.init_node('study_session', anonymous=True)
rate = rospy.Rate(10)

#Start log file
log_filename = 'Style_{}_Verbal_{}_Nonverbal_{}_{}.log'.format(ROBOT_STYLE,VERBAL_CADENCE, NONVERBAL_CADENCE, datetime.now().strftime("%Y-%m-%d--%H-%M-%S"))

#Initialize the feedback controller
feedback_controller = FeedbackController(True, log_filename, ROBOT_STYLE, VERBAL_CADENCE, NONVERBAL_CADENCE)

#Initialize evaluation object
exercise_eval = ExerciseEval(False, feedback_controller)

#For each exercise and set
for round_num in range(1, NUM_ROUNDS+1):

    #New Round
    exercise_eval.feedback_controller.logger.info('=====================================')
    exercise_eval.feedback_controller.logger.info('STARTING ROUND {} OF {}'.format(round_num, NUM_ROUNDS))
    exercise_eval.feedback_controller.logger.info('=====================================')

    # rospy.sleep(2)
    robot_message = "Round %s out of %s." % (round_num, NUM_ROUNDS)
    exercise_eval.feedback_controller.message(robot_message)

    for exercise_name in EXERCISE_LIST:
        for set_num in range(1, NUM_SETS+1):
            
            if exercise_name == 'lateral_raises' and set_num == NUM_SETS:
                is_final = True
            else:
                is_final = False
            
            #Start a new set
            exercise_eval.start_new_set(exercise_name, set_num)
            feedback_controller.start_new_set(set_num, NUM_SETS, exercise_name)

            inittime = datetime.now(timezone('EST'))
            exercise_eval.feedback_controller.logger.info('-------------------Recording!')
            start_message = False

            #Lower arm all the way down
            feedback_controller.move_right_arm('halfway', 'sides')

            #Stop between minimum and maximum time and minimum reps
            while (datetime.now(timezone('EST')) - inittime).total_seconds() < MAX_LENGTH:        
                
                #Robot says starting set
                if not start_message:
                    robot_message = "Start %s now" % (exercise_name.replace("_", " " ))
                    exercise_eval.feedback_controller.message(robot_message)
                    start_message = True
                    print('starting')

                exercise_eval.flag = True
                feedback_controller.flag = True

                #If number of reps is greater than 8 and they have been exercising at least the minimum length
                if len(exercise_eval.peaks[-1])-1 > MAX_REPS and (datetime.now(timezone('EST')) - inittime).total_seconds() > MIN_LENGTH:
                    break 

            exercise_eval.flag = False
            exercise_eval.feedback_controller.logger.info('-------------------Done with exercise')

            robot_message = "Almost done."
            exercise_eval.feedback_controller.message(robot_message)
            rospy.sleep(3)

            robot_message = "Rest."
            exercise_eval.feedback_controller.message(robot_message)
            if ROBOT_STYLE == 2:
                feedback_controller.change_expression('smile', 0.6, 4)
            elif ROBOT_STYLE == 3:
                feedback_controller.change_expression('smile', 0.8, 4)

            rest_start = datetime.now(timezone('EST'))

            #Raise arm all the way up
            feedback_controller.move_right_arm('sides', 'up')

            # if not is_final:
            #     halfway_message = False
            #     while (datetime.now(timezone('EST')) - rest_start).total_seconds() < REST_TIME:
                    
            #         #Print halfway done with rest here
            #         if (datetime.now(timezone('EST')) - rest_start).total_seconds() > REST_TIME/2 and not halfway_message:
            #             halfway_message = True
            #             robot_message = "Rest for {} more seconds.".format(int(REST_TIME/2))
            #             exercise_eval.feedback_controller.message(robot_message)
            # else:
            #     robot_message = "End of Round {}".format(round_num)
            #     exercise_eval.feedback_controller.message(robot_message)

            #     halfway_message = False
            #     while (datetime.now(timezone('EST')) - rest_start).total_seconds() < ROUND_REST_TIME:
                    
            #         #Print halfway done with rest here
            #         if (datetime.now(timezone('EST')) - rest_start).total_seconds() > ROUND_REST_TIME/2 and not halfway_message:
            #             halfway_message = True
            #             robot_message = "Rest for {} more seconds.".format(int(ROUND_REST_TIME/2))
            #             exercise_eval.feedback_controller.message(robot_message)
            
    data_filename = 'Style_{}_Verbal_{}_Nonverbal_{}.npz'.format(ROBOT_STYLE, VERBAL_CADENCE, NONVERBAL_CADENCE)
    
    exercise_eval.plot_angles()

    np.savez('src/quori_exercises/saved_data/{}'.format(data_filename),      
                            angles=exercise_eval.angles,
                            peaks=exercise_eval.peaks,
                            feedback=exercise_eval.feedback,
                            times=exercise_eval.times,
                            exercise_names=exercise_eval.exercise_name_list
                        )
    exercise_eval.feedback_controller.logger.info('Saved file {}'.format(data_filename))

    exercise_eval.feedback_controller.logger.handlers.clear()
    logging.shutdown()
    print('Done!')

plt.show()
