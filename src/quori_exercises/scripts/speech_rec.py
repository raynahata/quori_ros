#speech recogoition test 

# Python program to translate
# speech to text and text to speech


import speech_recognition as sr
from gtts import gTTS
import os 

def text_to_speech(text):
    text=str(text)
    
    text_quote = text.replace("'", "")
    text_quote = text_quote.replace("]", "")

    # Initialize gTTS with the text to convert
    speech = gTTS(text_quote, lang='en')
   

    # Save the audio file to a temporary file
    speech_file = 'speech.mp3'
    speech.save(speech_file)

    # Play the audio file
    os.system('afplay ' + speech_file)

# Initialize the recognizer 
r = sr.Recognizer() 

# Function to convert text to
# speech
# def SpeakText(command):
	
# 	# Initialize the engine
# 	engine = pyttsx3.init()
# 	engine.say(command) 
# 	engine.runAndWait()
	
	
# Loop infinitely for user to
# speak

while(1): 
	
	# Exception handling to handle
	# exceptions at the runtime
	try:
		
		# use the microphone as source for input.
		with sr.Microphone() as source2:
			
			# wait for a second to let the recognizer
			# adjust the energy threshold based on
			# the surrounding noise level 
			r.adjust_for_ambient_noise(source2, duration=0.5)
			
			#listens for the user's input 
			audio2 = r.listen(source2)
			
			# Using google to recognize audio
			MyText = r.recognize_google(audio2)
			#MyText = MyText.lower()

			print("Did you say ",MyText)
			text_to_speech(MyText)
			
	except sr.RequestError as e:
		print("Could not request results; {0}".format(e))
		
	except sr.UnknownValueError:
		print("unknown error occurred")
