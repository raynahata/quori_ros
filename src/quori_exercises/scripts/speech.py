from gTTS import gTTS
import os


folder_path = '/Users/raynahata/Desktop/Github/quori_ros/src/quori_exercises/speech_files/' 
if not os.path.exists(folder_path): 
    os.makedirs(folder_path) 

def text_to_speech(text):
    # Initialize gTTS with the text to convert
    speech = gTTS(text)

    # Save the audio file to a temporary file
    speech_file = 'speech.mp3'
    speech.save(speech_file)

    # Play the audio file
    os.system('afplay ' + speech_file)
