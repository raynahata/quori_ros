import time
from pynput import keyboard  # Using the keyboard library

# def on_press(key):
#     try:
    
#        return key
           
#     except AttributeError:
      
#         return
    
# def start_publishing(publish,key):
#         print("Start publishing. Press Enter to stop.")
#         while publish:
#             msg = "Hello, this is a test message!"
#             print(f"Publishing: {msg}")
#             time.sleep(1)

#             if key=='s':
#                 publish=False
#                 print("Stop publishing")
#                 break


# with keyboard.Listener(on_press=on_press) as listener:
#     listener.join()

#     start_publishing(True,listener)

def on_press(key):
    if key == keyboard.Key.enter:
        return False

with keyboard.Listener(on_press=on_press) as listener:
    for _ in range(50):
        print('still running...')
        time.sleep(1)
        if not listener.running:
            break