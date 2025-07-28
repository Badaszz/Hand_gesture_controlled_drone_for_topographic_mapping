
#...........................................................................
# Control Drone with Hand Gestures 
# By Yusuf Solomon Olumide Badaszzzz
#...........................................................................

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


import cv2
import mediapipe as mp
import hand_tracking_module2 as htm
import time
import os
from drone_control.DroneControlModuleMAVLINK import DroneKeyControl
from esp32cam import espcam as cam
import threading

hand = [None, None, None, None, None] #initializing the hand to be all None
hand_signs = {
    "INC": [0, 1, 0, 0, 0], #index finger up
    "DEC": [0, 1, 1, 0, 0], #index and middle finger up
    "RIGHT": [0, 0, 1, 1, 1], #index, middle and ring finger up
    "LEFT": [0, 1, 1, 1, 0], #thumb and middle finger down count=3
    "FORWARD": [0, 1, 1, 1, 1], #all fingers up except thumb
    "BACKWARD": [1, 1, 1, 1, 0], #all fingers up except pinky
    "TAKEOFF": [1, 1, 1, 1, 1], #all fingers up
    "LAND": [0, 0, 0, 0, 0] , #all fingers down
    "RTH" : [0, 1, 0, 0, 1], 
    "YAW_RIGHT" : [0, 0, 0, 0, 1], #all fingers down except  pinky
    "YAW_LEFT" : [1, 0, 0, 0, 0], #all fingers down except thumb
    "SWITCH" : [1, 1, 0, 0, 1], #index finger up count=1
    #"MAP AREA" : [1, 0, 0, 0, 1], #thumb and pinky up count=3
}


control_dict = {'TAKEOFF':'t','LAND':'b', 'RTH':'r','INC':'i',
                'DEC':'k','RIGHT':'l','LEFT':'j', 
                'YAW_LEFT':'q','YAW_RIGHT':'e', 'FORWARD':'w','BACKWARD':'s'}


def send_drone_command(command):
    # Function to send the command to the drone
    global in_action
    drone.press(control_dict[command])
    time.sleep(1)  # Simulate the time taken for the command to be executed
    in_action = False  # Reset the action flag after the command is sent
    
def run_camera(time):
    try:
        camera.run_flight_sequence(duration_sec=time)
    except Exception as e:
        print('camera error:', e)

wCam, hCam = 640, 480

cap = cv2.VideoCapture(0)
cap.set(3, wCam)
cap.set(4, hCam)

camIP = '192.168.4.3' # Replace with your ESP32-CAM IP
try:
    camera = cam.ESP32CamClient(camIP)
except Exception as e:
    print('error with cam:', e)
    
detector = htm.handDetector(detectionConfd = 0.7)

ft = [4, 8, 12, 16, 20] #finger tips
#ft[0] = thumb
#ft[1] = index finger
#ft[2] = middle finger
#ft[3] = ring finger
#ft[4] = pinky finger

in_action = False
droneIP = 'udp:172.29.144.1:14550' #'udp:127.0.0.1:14550'
print("Connecting to the drone...")
drone = DroneKeyControl(ip_address = droneIP)
if drone :
    print("Drone connection confirmed")
else:
    print("Drone Connection failed")

key = None

# present time 
pTime = 0

while True:
    success, img = cap.read()
    if not success:
        print("Camera not detected")
        continue
    img = detector.findLandmarks(img, draw = False)
    lmList = detector.findLmPositions(img, draw = False)
    
    if len(lmList) != 0:
        # check if index finger is up or down
        if lmList[ft[1]][2] < lmList[6][2]:
            hand[1] = 1
        else:
            hand[1] = 0

        # check if middle finger is up or down
        if lmList[ft[2]][2] < lmList[10][2]:
            hand[2] = 1
        else:
            hand[2] = 0
        
        # check if ring finger is up or down
        if lmList[ft[3]][2] < lmList[14][2]:
            hand[3] = 1
        else:
            hand[3] = 0
        
        # check if pinky finger is up or down
        if lmList[ft[4]][2] < lmList[18][2]:
            hand[4] = 1
        else:
            hand[4] = 0
        
        # check if thumb is open or closed
        if lmList[ft[0]][1] < lmList[3][1]:
            hand[0] = 0
        else:
            hand[0] = 1
    
    
    if in_action == False:        
        key = next((k for k, v in hand_signs.items() if v == hand), None) #only change key value when the drone is not in action
        
    txt = key if key else "No match"
    
    #img[200:0, 200:0] =  cv2.resize(overLayLst[0], (200, 200))
    img2 = cv2.flip(img,1)
    
    # computing the frames per second
    cTime = time.time()
    fps = 1/(cTime - pTime)
    pTime = cTime
    
    # writing the frames per second on the frame
    cv2.putText(img2, f'FPS: {int(fps)}', (400, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 255, 200), 3)
    
    if len(lmList) != 0 and not in_action:
        # display the action message
        cv2.putText(img2, txt, (70, 400), cv2.FONT_HERSHEY_PLAIN, 3, (255, 255, 200), 3)
        #print(txt)
    elif in_action :
        # display the action in progress message
        cv2.putText(img2, f"{key} in progress", (20, 400), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 200), 3)
        #print("Action in progress")
    else:
        cv2.putText(img2, "No hand detected", (20, 400), cv2.FONT_HERSHEY_PLAIN, 3, (255, 255, 200), 3)
        #print("No hand detected")
    if not in_action and key != "No match" and hand != [None,None, None, None, None] and key!=None: 
        in_action = True
        if key == "SWITCH": # switch to keyboard control
            cv2.putText(img2, "KeyBoard Control, NO VIDEO FEED press'esc' to exit ", (70, 400), cv2.FONT_HERSHEY_PLAIN, 3, (255, 255, 200), 3)
            print("KeyBoard Control, NO VIDEO FEED")
            print("press 'esc' to exit")
            cv2.imshow("Gesture Recognition", img2)
            drone.main() # run the keyboard control module main method
            in_action = False # reset the action flag
            continue
        # UNDER DEVELOPMENT
        # elif key == "MAP AREA": # map area
        #     print("Mapping area...")
        #     try:
        #         threading.Thread(target=run_camera, args=(10,)).start()  # Run camera for 10 seconds
        #         threading.Thread(target=send_drone_command, args=(key,)).start()
        #     except Exception as e:
        #         print(f"Error starting camera: {e}")
        #     in_action = False
        else:
            try:
                threading.Thread(target=send_drone_command, args=(key,)).start()
                print(f"{key} command sent")
            except Exception as e:
                print(f"Error sending command: {e}")
    cv2.imshow("Gesture Recognition", img2)
    hand = [None , None, None, None, None] # resetting the hand to be nothing
    if cv2.waitKey(1) & 0xFF == ord('p'):
        break