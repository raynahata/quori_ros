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
import speech as sp

#Change at the beginning of each session
PARTICIPANT_ID = '1'

folder_path = '/Users/raynahata/Desktop/Github/quori_ros/src/quori_exercises/intake_logs/' 
if not os.path.exists(folder_path): 
    os.makedirs(folder_path) 
# file_name = 'example.txt' 
# file_path = os.path.join(folder_path, file_name) 
# with open(file_path, 'w') as f: 
#     f.write('This is an example file.') 

#Start log file


intake_log_filename= 'Intake_{}.csv'.format(datetime.now().strftime("%Y-%m-%d--%H-%M-%S"))
log_fname = os.path.join(folder_path, intake_log_filename)


#Initialize logging
logger = logging.getLogger('logging')
logger.setLevel(logging.DEBUG)
fh = logging.FileHandler(format(log_fname))
fh.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
formatter=logging.Formatter(
    fmt='%(asctime)s.%(msecs)03d,%(message)s',
    datefmt='%Y-%m-%d,%H:%M:%S'
)
#formatter = logging.Formatter('%(asctime)s,%(message)s')
fh.setFormatter(formatter)
ch.setFormatter(formatter)
logger.addHandler(fh)
logger.addHandler(ch)
             

done_intake = False

#accessing the terminal input 
logger.info('Begin, {}'.format('Starting intake'))

while not done_intake:
    key, key_specific=ti.get_terminal_input()
    logger.info('Key, {}'.format(key))
    logger.info('Key Specific, {}'.format(key_specific))
    logger.info('Message, {}'.format(INTAKE_MESSAGES[key][key_specific]))
    #logger.info(key)
    #logger.info(key_specific)
    #logger.info(INTAKE_MESSAGES[key][key_specific])
    cont=input("Quit or Continue?")
    if cont == "":
        logger.info('Begin speaking')
        sp.text_to_speech(INTAKE_MESSAGES[key][key_specific])
        logger.info('End speaking')
        continue
    elif cont=="quit":
        done_intake=True
        logger.info('Quit')
    print("")


logger.handlers.clear()
logging.shutdown()
print('Done!')
