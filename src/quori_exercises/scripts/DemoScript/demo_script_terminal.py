#Demo Script

from datetime import datetime
from demo_messages import *
import logging
import demo_input as ti
import os
from gtts import gTTS
import demo_speech as sp
from pynput import keyboard


def get_message(name="Edward"):
    key=ti.get_key()

    key_specific=ti.get_terminal_input(key)

    if key_specific=="Greeting":
        message=DEMO_MESSAGES[key][key_specific][0].format(name=name)
        return message
    if key_specific=="back":
        return get_message()
    elif key_specific=="quit":
        return "quit"
    
        
    return DEMO_MESSAGES[key][key_specific]

#Change at the beginning of each session
#PARTICIPANT_NAME = 'Edward'
#PARTICIPANT_NAME = 'Edith'
#PARTICIPANT_NAME = 'Clara'
PARTICIPANT_NAME = 'Michelle'

done_intake = False

while not done_intake:
    message=get_message(PARTICIPANT_NAME)
    

    if message == "quit":
        done_intake=True
        break
    print(message)
    speak=input("Press enter to speak, else type b to go back")
    if speak == "":
        print("Begin speaking")
        sp.text_to_speech(message)
        print("End speaking")
        
    elif speak=="b":
        continue

    
    cont=input("Quit or Continue?")
    if cont=="q":
        done_intake=True
    elif cont=="":
        continue
   
logging.shutdown()
print('Done!')



