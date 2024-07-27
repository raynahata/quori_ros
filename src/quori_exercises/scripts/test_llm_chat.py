
#import getpass
import os
from langchain_mistralai import ChatMistralAI
from langchain_core.messages import HumanMessage
from langchain_core.output_parsers import StrOutputParser
from langchain_core.messages import AIMessage
#from langchain_community.llms import Ollama
from langchain_community.chat_message_histories import ChatMessageHistory
from langchain_core.chat_history import BaseChatMessageHistory
from langchain_core.runnables.history import RunnableWithMessageHistory
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
#from pynput import keyboard
from langchain_community.chat_models import ChatOllama
import re




#setup of the langchain
os.environ["LANGCHAIN_TRACING_V2"] = "true"
os.environ["LANGCHAIN_API_KEY"] = "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"
model = ChatOllama(model="llama2",temperature=0) #use mistral as model


store = {}
config = {"configurable": {"session_id": "test1"}} #session ID, resets every time you make a new session ID

def get_session_history(session_id: str) -> BaseChatMessageHistory:
    if session_id not in store:
        store[session_id] = ChatMessageHistory()
    return store[session_id]

#clean up response 
def remove_between_asterisks(text):
    # This regular expression will match any text between two asterisks
    return re.sub(r'\*.*?\*', '', text)

#model that you want to use


#use rag to extract examples insteat of the prompt 
#custom prompt for exercise robot 
prompt = ChatPromptTemplate.from_messages(
    [
        (
            "system",
            "You are an exercise coach robot named Quori. Your goal is to lead the participant through the introduction of the exercises and give feedback when they are exercising. The exercises that they will be doing today are bicep curls and lateral raises. Before you start your exercise session, you will introduce the exercises and ask a series of evaluation questions. If you don't know the answer, say that you don't know. You are polite, respectful, and aim to provide concise responses of less than 20 words. The exercise session that they will be doing is two rounds of 4 sets each. The first two sets in each round will be bicep curls and the second two will be lateral raises. Each set will be for 40 seconds. Here is the flow of the session: 1. Say hello, say your name, and ask how they are doing today 2. Tell them that before you explain the exercises you will be asking some questions 3. Ask how old they are 4. Ask if exercise is something that they choose to do 5. Ask if they feel like someone else is making them exercise such as their doctor or family member  6. Ask them what their energy level is on a scale of 1-10 7. Describe the exercise session to them 8. Ask if they have any questions and answer them. 8. Ask them if they prefer a strict coaching style or a encouraging coaching style. If they ask for examples, give examples. 9. Say you are going to start the exercise session then exit. ",
        ),
        MessagesPlaceholder(variable_name="messages"),
    ]
)

#stringing together the model and the custom prompt 
chain = prompt | model | StrOutputParser()

#name of the user 
name="Rayna"

#invoke a response 
#response = chain.invoke({"messages": [HumanMessage(content="hi! I'm " + name)]})


with_message_history = RunnableWithMessageHistory(chain, get_session_history)

response = with_message_history.invoke(
    [HumanMessage(content="Hi! I'm" + name)],
    config=config,

)

clean_response=remove_between_asterisks(response)
print(clean_response)

done_chat=False

while not done_chat:
    content=input("Put in human message:")
    response = with_message_history.invoke(
    [HumanMessage(content= content)],
    config=config,
    )

    if content=="bye":
        done_chat=True
        break
    clean_response=remove_between_asterisks(response)
    print(clean_response)


