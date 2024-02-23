#This is the file to run to start the intake proccess

#!/usr/bin/env python3
# import rospy
# import rosbag
import numpy
import sys 
import matplotlib.pyplot as plt
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
from pynput import keyboard

def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(
            key.char))
    except AttributeError:
        print('special key {0} pressed'.format(
            key))

def on_release(key):
    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False

#Change at the beginning of each session
PARTICIPANT_ID = '1'

folder_path = '/Users/raynahata/Desktop/Github/quori_ros/src/quori_exercises/intake_logs/' 
if not os.path.exists(folder_path): 
    os.makedirs(folder_path) 

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

# with keyboard.Listener(
#         on_press=on_press,
#         on_release=on_release) as listener:
#     listener.join()

def get_message():
    key=ti.get_key()
    logger.info('Key, {}'.format(key))
    # if key=="quit":
    #     done_intake=True
    #     logger.info('Quit')
    #     return

    key_specific=ti.get_terminal_input(key)
    if key_specific=="back":
        logger.info('Key Specific, {}'.format(key_specific))
        return get_message()
    logger.info('Key Specific, {}'.format(key_specific))
    logger.info('Message, {}'.format(INTAKE_MESSAGES[key][key_specific]))

    return INTAKE_MESSAGES[key][key_specific]
    

while not done_intake:
    message=get_message()
    #print("here")

    speak=input("Press enter to speak, else type back to go back")

    if speak == "":
        #logger.info('Begin speaking')
        logger.info('Begin speaking,{}'.format(message))
        sp.text_to_speech(message)
        logger.info('End speaking')
        
    elif speak=="back":
        #done_intake=True
        #logger.info('Quit')
        continue

    cont=input("Quit or Continue?")
    if cont=="quit":
        done_intake=True
        logger.info('Quit')
    elif cont=="":
        continue
    # key=ti.get_key()
   # logger.info('Key, {}'.format(key))

    
   # key_specific=ti.get_terminal_input(key)
    #if key_specific=="back":
      #  key=ti.get_key()
        #logger.info('Key, {}'.format(key))


  
    #logger.info('Key Specific, {}'.format(key_specific))
    #logger.info(key)
    #logger.info(key_specific)
    #logger.info(INTAKE_MESSAGES[key][key_specific])
    #logger.info('Key, {}'.format(key))
    #logger.info('Key Specific, {}'.format(key_specific))
    #logger.info('Message, {}'.format(INTAKE_MESSAGES[key][key_specific]))


logger.handlers.clear()
logging.shutdown()
print('Done!')

#TODO: Add a back option logging  --> option 1: create new CSV file
# Option 2: Add keyboard listener to the intake file 

