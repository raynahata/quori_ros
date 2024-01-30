#!/usr/bin/env python3
import rospy
import rosbag
import numpy as np
import sys 
import matplotlib.pyplot as plt
from config import *
from ExerciseController import ExerciseController
from datetime import datetime
from pytz import timezone
from intake_messages import *
import logging
import time
import terminal_input as ti

#Change at the beginning of each session
PARTICIPANT_ID = '1'


#Initialize ROS node
rospy.init_node('study_session', anonymous=True)
rate = rospy.Rate(10)

#Start log file
intake_log_filename= 'Intake_{}.log'.format(datetime.now().strftime("%Y-%m-%d--%H-%M-%S"))


#Initialize evaluation object
#intake_controller = ExerciseController(False, intake_log_filename)

#Initialize logging
logger = logging.getLogger('logging')
logger.setLevel(logging.DEBUG)
fh = logging.FileHandler('src/quori_exercises/saved_intake_logs/{}'.format(intake_log_filename))
fh.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
fh.setFormatter(formatter)
ch.setFormatter(formatter)
logger.addHandler(fh)
logger.addHandler(ch)

done_intake = False
#accessing the terminal input 






#For each exercise and set
for round_num in range(1, NUM_ROUNDS+1):

    #New Round
    controller.logger.info('=====================================')
    controller.logger.info('STARTING ROUND {} OF {}'.format(round_num, NUM_ROUNDS))
    controller.logger.info('=====================================')

    # rospy.sleep(2)
    robot_message = "Round %s out of %s." % (round_num, NUM_ROUNDS)
    controller.message(robot_message)

    for exercise_name in EXERCISE_LIST:
        for set_num in range(1, NUM_SETS+1):
            
            if exercise_name == 'lateral_raises' and set_num == NUM_SETS:
                is_final = True
            else:
                is_final = False
            
            #Start a new set
            controller.start_new_set(exercise_name, set_num)

            inittime = datetime.now(timezone('EST'))
            controller.logger.info('-------------------Recording!')
            start_message = False

            #Lower arm all the way down
            controller.move_right_arm('halfway', 'sides')

            #Stop between minimum and maximum time and minimum reps
            while (datetime.now(timezone('EST')) - inittime).total_seconds() < MAX_LENGTH:        
                
                #Robot says starting set
                if not start_message:
                    robot_message = "Start %s now" % (exercise_name.replace("_", " " ))
                    controller.message(robot_message)
                    start_message = True

                controller.flag = True

                #If number of reps is greater than 8 and they have been exercising at least the minimum length
                if len(controller.peaks[-1])-1 > MAX_REPS and (datetime.now(timezone('EST')) - inittime).total_seconds() > MIN_LENGTH:
                    break 

            controller.flag = False
            controller.logger.info('-------------------Done with exercise')

            robot_message = "Almost done."
            controller.message(robot_message)
            rospy.sleep(3)

            robot_message = "Rest."
            controller.message(robot_message)
            controller.change_expression('smile', controller.start_set_smile, 4)

            rest_start = datetime.now(timezone('EST'))

            #Raise arm all the way up
            controller.move_right_arm('sides', 'up')

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
    
   

    controller.logger.info('Saved file {}'.format(data_filename))

    controller.logger.handlers.clear()
    logging.shutdown()
    print('Done!')

plt.show()
