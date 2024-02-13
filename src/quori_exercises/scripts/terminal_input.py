#Current iteration of the terminal inputs
from intake_messages import *

def get_key():
    try:
        section_num_input=input("What section are you on? \n 1. Introduction \n 2.Consent \
                    \n 3. Evaluation \n 4. Exercise explanation  \n 5. Coach Type \n 6. Fall Back \n")
        
        if section_num_input == "1": key="Introduction"
        elif section_num_input == "2": key= "Consent"
        elif section_num_input == "3": key= "Evaluation"
        elif section_num_input == "4": key="Exercise explanation"
        elif section_num_input == "5": key= "Coach type"
        elif section_num_input == "6": key= "Fall back"
        return key
        
        
    except:
        print("Invalid input try again")
        return get_key()
        
    
def get_key_intro():
    response_num=input("1. Greeting \n 2. Fun \n 3. Response positive \n 4. Response negative \n")
    try:
        if response_num == "1": key_specific="Greeting"
        elif response_num== "2": key_specific="Fun"
        elif response_num == "3": key_specific="Response positive"
        elif response_num == "4": key_specific="Response negative"
        return key_specific
    except:
        print("Invalid input")
        return get_key_intro()
    

def get_key_consent():
    response_num=input("1. Name \n 2. Age \n 3. Explanation \n")
    try:
        if response_num == "1": key_specific="Name"
        elif response_num== "2": key_specific="Age"
        elif response_num == "3": key_specific="Explanation"
       # elif response_num == "4": key_specific="Sign consent"
        return key_specific
    except:
        print("Invalid input")
        return get_key_consent()
    

def get_key_evaluation():
    response_num=input("1. Pain \n 2. Energy Level \n")
    try:
        if response_num == "1": key_specific="Pain"
        elif response_num== "2": key_specific="Energy Level"
        return key_specific
    except:
        print("Invalid input")
        return get_key_evaluation()
    

def get_key_exercise():
    response_num=input("1. Start explanation \n 2. Explain exercise routine \n 3. Dumbbells \n")
    try:
        if response_num == "1": key_specific="Start explanation"
        elif response_num== "2": key_specific="Explain exercise routine"
        elif response_num == "3": key_specific="Dumbells"
        return key_specific
    except:
        print("Invalid input")
        return get_key_exercise()
    

def get_key_coach_type():
    response_num=input("Ask coach type question?")
    try:
        if response_num == "": key_specific="Ask type"
        elif response_num== "quit": key_specific="quit"
        return key_specific
    except:
        print("Invalid input")
        return get_key_coach_type()
    

def get_key_fall_back():
    response_num=input("1. No answer \n 2. Repeat \n 3. Clarify \n")
    try:
        if response_num == "1": key_specific="No answer"
        elif response_num== "2": key_specific="Repeat"
        elif response_num == "3": key_specific="Clarify"
        return key_specific
    except:
        print("Invalid input")
        return get_key_fall_back()
   

def get_terminal_input():
    key=get_key()
    if key == "Introduction":
        key_specific=get_key_intro()
    elif key == "Consent":
        key_specific=get_key_consent()
    elif key == "Evaluation":
        key_specific=get_key_evaluation()
    elif key == "Exercise explanation":
        key_specific=get_key_exercise()
    elif key == "Coach type":
        key_specific=get_key_coach_type()
    elif key == "Fall back":
        key_specific=get_key_fall_back()
    return key, key_specific

    
while True:
    key, key_specific=get_terminal_input()
    print(key, key_specific)
    print(INTAKE_MESSAGES[key][key_specific])
    cont=input("Quit or Continue?")
    if cont == "":
        continue
    elif cont=="quit":
        break
    print("")
    
    

    