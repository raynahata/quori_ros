from gtts import gTTS
import os
import intake_messages as im
from io import BytesIO


folder_path = '/Users/raynahata/Desktop/Github/quori_ros/src/quori_exercises/speech_files/' 
if not os.path.exists(folder_path): 
    os.makedirs(folder_path) 

def text_to_speech(text):
    text=str(text)
    #chars_to_remove = ["'", '"', "(", ")", "[", "]", "{", "}"]
    
    text_quote = text.replace("'", "")
    text_quote = text_quote.replace("]", "")
    # Initialize gTTS with the text to convert
    speech = gTTS(text_quote, lang='en')
   # fp = BytesIO()
   # speech.write_to_fp(fp)
    #fp.seek(0)

    # Save the audio file to a temporary file
    speech_file = 'speech.mp3'
    speech.save(speech_file)

    # Play the audio file
    os.system('afplay ' + speech_file)


def main():
    text=im.INTAKE_MESSAGES["Introduction"]["Greeting"]
    # text=str(text)
    # text_quote = text.replace("'", "")
    #text = "Hello, my name is Quori. I am a robot designed to help you with your exercises. I am excited to work with you today."
    text_to_speech(text)



