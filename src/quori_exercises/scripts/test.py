import getpass
import os
from langchain_mistralai import ChatMistralAI
from langchain_core.messages import HumanMessage
from langchain_core.messages import AIMessage




os.environ["LANGCHAIN_TRACING_V2"] = "true"
os.environ["LANGCHAIN_API_KEY"] = getpass.getpass()
os.environ["MISTRAL_API_KEY"] = getpass.getpass()

model = ChatMistralAI(model="mistral-large-latest")

name="Rayna"

model.invoke(
    [
        HumanMessage(content="Hi! I'm " + name),
        AIMessage(content="Hello " + name + "! My name is Quori. How are you today?"),
        HumanMessage(content="What's my name?"),
    ]
)


