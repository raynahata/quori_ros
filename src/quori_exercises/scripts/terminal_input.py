from intake_messages import *


def get_terminal_input():
    section=input("What section are you on? \n 1. Introduction \n 2.Consent \n 3. Evaluation \n 4. Exercise  \n 5. Coach Type \n 6. Fall Back \n")
                  
    m=[]
    if section == "1":
        m.append("Introduction")
        response=input("1. Greeting \n 2. Response positive \n 3. Response negative \n")
        if response == "1": m.append("Greeting")
        elif response == "2": m.append("Fun")
        elif response == "3": m.append("Response positive")
        elif response == "4": m.append("Response negative")
    
    elif section == "2":
        m.append("Consent")
        response=input("1. Name \n 2. Age \n 3. Explanation \n 4. Sign consent \n")
        if response == "1": m.append("Name")
        elif response == "2": m.append("Age")
        elif response == "3": m.append("Explanation")
        elif response == "4": m.append("Sign consent")
    
    elif section == "3":
        m.append("Evaluation")
        response=input("1. Pain \n 2. Energy Level \n")
        if response == "1": m.append("Pain")
        elif response == "2": m.append("Energy Level")
    
    elif section == "4":
        m.append("Exercise")
        response=input("1. Start explanation \n 2. Explain exercise routine \n 3. Dumbells \n")
        if response == "1": m.append("Start explanation")
        elif response == "2": m.append("Explain exercise routine")
        elif response == "3": m.append("Dumbells")


    elif section == "5":
        m.append("Coach type")
        response=input("Ask coach type question?")
        if response=="": m.append("Ask type")
        elif response=="quit": m.append("quit")
    
    elif section == "6": 
        m.append("Fall Back")   
        response=input("1. No answer \n 2. Repeat \n 3. Clarify \n")
        if response == "1": m.append("No answer")
        elif response == "2": m.append("Repeat")
        elif response == "3": m.append("Clarify")
        
    return m 


    
# while True:
#     m= get_terminal_input()
#     if m[1]=="quit":
#         break
#     print(INTAKE_MESSAGES[m[0]][m[1]])  
#     if input("Pressing enter will send the command to the robot! ") == "":
#         print(INTAKE_MESSAGES[m[0]][m[1]]) #this would be the push to robot commmand 
    
#     elif input() =="quit":
#         break
#     else:
#         if input() == "back":
#             m=get_terminal_input()
    
#     if input("Press 2 to Quit, otherwise enter\n") =="2":
#         break
    
    

    