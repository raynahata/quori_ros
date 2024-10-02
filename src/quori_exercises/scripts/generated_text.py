from langchain.chains import RetrievalQA
from langchain.embeddings import HuggingFaceEmbeddings
from langchain.callbacks.streaming_stdout import StreamingStdOutCallbackHandler
from langchain.vectorstores import Chroma
from langchain.llms import Ollama
import chromadb
import os
import argparse
import time

model = os.environ.get("MODEL", "mistral")

embeddings_model_name = os.environ.get("EMBEDDINGS_MODEL_NAME", "all-MiniLM-L6-v2")
persist_directory = os.environ.get("PERSIST_DIRECTORY", "db")
target_source_chunks = int(os.environ.get('TARGET_SOURCE_CHUNKS',4))


def load_txt(filename):
    with open(filename, 'rt') as of:
        text_file = of.readlines()
    text = ' '.join(text_file)
    text = text.replace('\n', ' ').replace('  ', ' ')
    return text

template = ut.load_txt('prompt.txt')

# 1. start exercise sessions 
# start_session_prompt= "Start exercise session "

# 2. generate a response 
# generate_response_prompt= prompt(start_exercise_session)

#3. query the response 
# query. append (generate_response_prompt)

#4. listen for the speaker 
# generate response (speaker response)

#5 query the response
# query.append (generate_response_prompt)

