from intake_messages import *


def get_terminal_input():
    section=input("What section are you on? \n 1. Introduction \n 2. Fall Back \n 3. Exercise Explanation \n 4. Coach Type \n")
    m=[]
    if section == "1":
        m.append("Introduction")
        response=input("1. Greeting \n 2. Response positive \n 3. Response negative \n")
        if response == "1": m.append("Greeting")
        elif response == "2": m.append("Response positive")
        elif response == "3": m.append("Response negative")
        
    
    elif section == "2": 
        m.append("Fall Back")   
        response=input("1. No answer \n 2. Repeat \n 3. Clarify \n")
        if response == "1": m.append("No answer")
        elif response == "2": m.append("Repeat")
        elif response == "3": m.append("Clarify")

    
    elif section == "3":
        m.append("Exercise explanation")
        response=input("1. Lateral raises \n 2. Bicep curls \n")
        if response == "1": m.append("Lateral raises")
        elif response == "2":m.append("Bicep curls")
     
    elif section == "4":
        m.append("Coach type")
        response=input("Ask coach type question?")
        if response=="": m.append("Ask type")
        elif response=="quit": m.append("quit")
       
        
    return m 

    
while True:
    m= get_terminal_input()
    if m[1]=="quit":
        break
    print(INTAKE_MESSAGES[m[0]][m[1]])  
    if input("Pressing enter will send the command to the robot! ") == "":
        print(INTAKE_MESSAGES[m[0]][m[1]]) #this would be the push to robot commmand 
    
    elif input() =="quit":
        break
    else:
        if input() == "back":
            m=get_terminal_input()
    