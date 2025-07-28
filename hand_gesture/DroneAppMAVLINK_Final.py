#...........................................................................
# Control Drone with Hand Gestures, this is an APP
# By Yusuf Solomon Olumide Badaszzzz
#...........................................................................

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


import tkinter as tk
from tkinter import messagebox
import threading
import time
import cv2
from PIL import Image, ImageTk
from esp32cam.espcam import ESP32CamClient
from hand_gesture import hand_tracking_module2 as htm
from drone_control.DroneControlModuleMAVLINK import DroneKeyControl

class DroneAppMAVLINK:
    def __init__(self, master):
        self.master = master
        master.title("MAVLink Drone App")
        master.geometry("950x550")
        master.resizable(False, False)

        self.cam_client = ESP32CamClient("192.168.4.3")
        try:
            self.cam_client.connect()
        except Exception as e:
            print("Camera connection error:", e)

        self.detector = htm.handDetector(detectionConfd=0.7, mode=True)
        self.cap = None
        self.hand_signs = {
            "INC": [0, 1, 0, 0, 0],
            "DEC": [0, 1, 1, 0, 0],
            "RIGHT": [0, 0, 1, 1, 1],
            "LEFT": [0, 1, 1, 1, 0],
            "FORWARD": [0, 1, 1, 1, 1],
            "BACKWARD": [1, 1, 1, 1, 0],
            "TAKEOFF": [1, 1, 1, 1, 1],
            "LAND": [0, 0, 0, 0, 0],
            "RTH": [0, 1, 0, 0, 1],
            "YAW_RIGHT": [0, 0, 0, 0, 1],
            "YAW_LEFT": [1, 0, 0, 0, 0],
            "SWITCH": [1, 1, 0, 0, 1]
        }
        self.control_map = {
            'TAKEOFF':'t','LAND':'b', 'RTH':'r','INC':'i',
            'DEC':'k','RIGHT':'l','LEFT':'j',
            'YAW_LEFT':'q','YAW_RIGHT':'e', 'FORWARD':'w','BACKWARD':'s'
        }
        self.hand = [None] * 5
        self.ft = [4, 8, 12, 16, 20]
        try:
            self.drone = DroneKeyControl(ip_address='udp:127.0.0.1:14550')
        except Exception as e:
            print("[DRONE ERROR]", e)
        self.in_action = False

        tk.Label(master, text="MAVLink Drone Controller", font=("Arial", 16, "bold")).pack(pady=10)
        tk.Button(master, text="Start Gesture", command=self.start_gesture).pack(pady=2)
        tk.Button(master, text="Stop Gesture", command=self.stop_gesture).pack(pady=2)
        tk.Button(master, text="Start Capture", command=lambda: threading.Thread(target=self.cam_client.start_capture).start()).pack(pady=2)
        tk.Button(master, text="Stop Capture", command=lambda: threading.Thread(target=self.cam_client.stop_capture).start()).pack(pady=2)
        tk.Button(master, text="Exit", command=self.exit_app).pack(pady=10)
        self.video_label = tk.Label(master)
        self.video_label.pack()

    def send_command(self, key):
        try:
            self.drone.press(self.control_map[key])
        except Exception as e:
            print("Command error:", e)
        self.in_action = False

    def map_area(self):
        try:
            threading.Thread(target=self.cam_client.start_capture).start()
            threading.Thread(target=self.send_command, args=("RTH",)).start()
            print("Mapping initiated...")
        except Exception as e:
            print("Mapping error:", e)
        self.in_action = False

    def update_frame(self):
        success, img = self.cap.read()
        if not success: return

        img = self.detector.findLandmarks(img, draw=True)
        lmList = self.detector.findLmPositions(img, draw=False)
        if lmList:
            self.hand[1] = int(lmList[self.ft[1]][2] < lmList[6][2])
            self.hand[2] = int(lmList[self.ft[2]][2] < lmList[10][2])
            self.hand[3] = int(lmList[self.ft[3]][2] < lmList[14][2])
            self.hand[4] = int(lmList[self.ft[4]][2] < lmList[18][2])
            self.hand[0] = int(lmList[self.ft[0]][1] > lmList[3][1])
            key = next((k for k, v in self.hand_signs.items() if v == self.hand), None)
            if key and not self.in_action:
                self.in_action = True
                if key == "RTH":
                    threading.Thread(target=self.map_area).start()
                else:
                    threading.Thread(target=self.send_command, args=(key,)).start()

        img = cv2.cvtColor(cv2.flip(img, 1), cv2.COLOR_BGR2RGB)
        imgtk = ImageTk.PhotoImage(Image.fromarray(img))
        self.video_label.imgtk = imgtk
        self.video_label.configure(image=imgtk)
        self.master.after(10, self.update_frame)

    def start_gesture(self):
        if not self.cap:
            self.cap = cv2.VideoCapture(0)
            self.update_frame()

    def stop_gesture(self):
        if self.cap:
            self.cap.release()
            self.cap = None
            self.video_label.config(image='')

    def exit_app(self):
        if self.cap:
            self.cap.release()
        self.cam_client.close()
        self.master.quit()


if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = DroneAppMAVLINK(root)
        root.mainloop()
    except Exception as e:
        print("[FATAL ERROR]", e)
        root.destroy()
