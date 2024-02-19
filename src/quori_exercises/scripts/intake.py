#This is the file to run to start the intake proccess

#!/usr/bin/env python3
# import rospy
# import rosbag
#import numpy
import sys 
#import matplotlib.pyplot as plt
#from config import *
#from ExerciseController import ExerciseController
from datetime import datetime
#from pytz import timezone
from intake_messages import *
import logging
import time
import terminal_input as ti
import os
from gtts import gTTS
#Change at the beginning of each session
PARTICIPANT_ID = '1'

folder_path = '/Users/raynahata/Desktop/Github/quori_ros/src/quori_exercises/intake_logs' 
if not os.path.exists(folder_path): 
    os.makedirs(folder_path) 
# file_name = 'example.txt' 
# file_path = os.path.join(folder_path, file_name) 
# with open(file_path, 'w') as f: 
#     f.write('This is an example file.') 





#Start log file

intake_log_filename= 'Intake_{}.log'.format(datetime.now().strftime("%Y-%m-%d--%H-%M-%S"))
#logging.basicConfig(filename=intake_log_filename,filemode='w', level=logging.INFO)

#logging.basicConfig(filename='app.log', filemode='w', format='%(name)s - %(levelname)s - %(message)s')


#Initialize evaluation object
#intake_controller = ExerciseController(False, intake_log_filename)
#file = open('myfile.txt', 'w')
#file.close()

#Initialize logging
logger = logging.getLogger('logging')
logger.setLevel(logging.DEBUG)
fh = logging.FileHandler('src/quori_exercises/intake_logs/{}'.format(intake_log_filename))
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
logger.info('Starting intake')

while not done_intake:
    key, key_specific=ti.get_terminal_input()

    logger.info('Key: {}'.format(key))
    logger.info('Key Specific: {}'.format(key_specific))
    logger.info('Message: {}'.format(INTAKE_MESSAGES[key][key_specific]))
    cont=input("Quit or Continue?")
    if cont == "":
        continue
    elif cont=="quit":
        done_intake=True
    
  
    print("")
    


# while True:
#     key, key_specific=ti.get_terminal_input()
#     #print(key, key_specific)
#     #print(INTAKE_MESSAGES[key][key_specific])
#     cont=input("Quit or Continue?")
#     if cont == "":
#         continue
#     elif cont=="quit":
#         break
#     print("")

# logger.info('Starting intake')
# while not done_intake:
#     #key, key_specific = ti.get_terminal_input()
#     logger.info('Key: {}'.format(key))
#     logger.info('Key Specific: {}'.format(key_specific))
#     logger.info('Message: {}'.format(INTAKE_MESSAGES[key][key_specific]))
#     #robot_message = INTAKE_MESSAGES[key][key_specific]
#     #logger.info('Robot message: {}'.format(robot_message))
#     #intake_controller.message(robot_message)
#     done_intake = True
#     #rospy.sleep(2)

logger.handlers.clear()
logging.shutdown()
print('Done!')
