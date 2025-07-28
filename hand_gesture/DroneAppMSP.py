import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


import tkinter as tk
from tkinter import messagebox
import threading
import os
import time
import cv2
from PIL import Image, ImageTk
from esp32cam.espcam import ESP32CamClient
from hand_gesture import hand_tracking_module2 as htm  # Make sure this exists


class DroneCamApp:
    def __init__(self, master):
        self.master = master
        master.title("Drone & ESP32-CAM Control Panel")
        master.geometry("950x550")
        master.resizable(False, False)

        # ESP32 Camera client
        self.cam_client = ESP32CamClient("192.168.4.3")  # Update IP if needed
        self.cam_client.connect()

        # Gesture recognition setup
        self.cap = None
        self.detector = htm.handDetector(detectionConfd=0.7, mode=True)
        self.hand = [None, None, None, None, None]
        self.ft = [4, 8, 12, 16, 20]
        self.hand_signs = {
            "INC": [0, 1, 0, 0, 0],
            "DEC": [0, 1, 1, 0, 0],
            "RIGHT": [0, 0, 1, 1, 1],
            "LEFT": [0, 1, 1, 1, 0],
            "FORWARD": [0, 1, 1, 1, 1],
            "BACKWARD": [1, 1, 1, 1, 0],
            "TAKEOFF": [1, 1, 1, 1, 1],
            "LAND": [0, 0, 0, 0, 0],
            "MAP_AREA": [0, 1, 0, 0, 1],
            "YAW_RIGHT": [0, 0, 0, 0, 1],
            "YAW_LEFT": [1, 0, 0, 0, 0],
            "SWITCH": [1, 1, 0, 0, 1]
        }
        self.pTime = 0

        # --- Title ---
        tk.Label(master, text="Drone & Camera Controller", font=("Arial", 16, "bold")).grid(row=0, column=0, columnspan=2, pady=10)

        # --- Gesture Control Buttons ---
        gesture_frame = tk.LabelFrame(master, text="Gesture Recognition", font=("Arial", 10, "bold"))
        gesture_frame.grid(row=1, column=1, sticky="n", padx=10, pady=10)

        tk.Button(gesture_frame, text="Start Gesture Control", command=self.start_gesture, bg="green", fg="white", width=25).pack(pady=2)
        tk.Button(gesture_frame, text="Stop Gesture Control", command=self.stop_gesture, bg="red", fg="white", width=25).pack(pady=2)

        # --- ESP32-CAM Controls ---
        cam_frame = tk.LabelFrame(master, text="ESP32-CAM Control", font=("Arial", 10, "bold"))
        cam_frame.grid(row=2, column=1, sticky="n", padx=10)

        tk.Button(cam_frame, text="Start Capture", command=self.start_capture, width=25).pack(pady=2)
        tk.Button(cam_frame, text="Stop Capture", command=self.stop_capture, width=25).pack(pady=2)
        tk.Button(cam_frame, text="Download Images", command=self.download_images, width=25).pack(pady=2)
        tk.Button(cam_frame, text="Clear Storage", command=self.clear_storage, width=25).pack(pady=2)

        # --- Exit Button ---
        tk.Button(master, text="Exit", command=self.on_exit, bg="black", fg="white", width=25).grid(row=3, column=1, pady=20)

        # --- Video Feed (Left Column) ---
        self.video_label = tk.Label(master)
        self.video_label.grid(row=1, column=0, rowspan=4, padx=10, pady=10)

    def update_gesture_frame(self):
        success, img = self.cap.read()
        if not success:
            print("Camera not detected")
            return

        img = self.detector.findLandmarks(img, draw=True)
        lmList = self.detector.findLmPositions(img, draw=False)

        if len(lmList) != 0:
            self.hand[1] = int(lmList[self.ft[1]][2] < lmList[6][2])
            self.hand[2] = int(lmList[self.ft[2]][2] < lmList[10][2])
            self.hand[3] = int(lmList[self.ft[3]][2] < lmList[14][2])
            self.hand[4] = int(lmList[self.ft[4]][2] < lmList[18][2])
            self.hand[0] = int(lmList[self.ft[0]][1] > lmList[3][1])

        key = next((k for k, v in self.hand_signs.items() if v == self.hand), None)
        txt = key if key else "No match"

        cTime = time.time()
        fps = 1 / (cTime - self.pTime) if (cTime - self.pTime) > 0 else 0
        self.pTime = cTime

        img = cv2.flip(img, 1)
        cv2.putText(img, f'FPS: {int(fps)}', (400, 70), cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 200), 2)
        cv2.putText(img, txt, (70, 400), cv2.FONT_HERSHEY_PLAIN, 3, (255, 255, 200), 2)

        # Show frame in GUI
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(img_rgb)
        imgtk = ImageTk.PhotoImage(image=img_pil)
        self.video_label.imgtk = imgtk
        self.video_label.configure(image=imgtk)

        self.hand = [None, None, None, None, None]
        self.master.after(10, self.update_gesture_frame)

    def start_gesture(self):
        if self.cap is None:
            self.cap = cv2.VideoCapture(0)
            self.cap.set(3, 640)
            self.cap.set(4, 480)
            self.update_gesture_frame()

    def stop_gesture(self):
        if self.cap:
            self.cap.release()
            self.cap = None
            self.video_label.config(image='')
            messagebox.showinfo("Stopped", "Gesture control stopped.")
        else:
            messagebox.showinfo("Not Running", "Gesture control is not running.")

    def start_capture(self):
        threading.Thread(target=self.cam_client.start_capture).start()
        messagebox.showinfo("Camera", "Started image capture.")

    def stop_capture(self):
        threading.Thread(target=self.cam_client.stop_capture).start()
        messagebox.showinfo("Camera", "Stopped image capture.")

    def download_images(self):
        threading.Thread(target=self.cam_client.download_images).start()
        messagebox.showinfo("Download", "Download started. Check 'downloaded_images' folder.")

    def clear_storage(self):
        threading.Thread(target=self.cam_client.clear_storage).start()
        messagebox.showinfo("Camera", "Camera storage cleared.")

    def on_exit(self):
        if self.cap:
            self.cap.release()
        self.cam_client.close()
        self.master.quit()


if __name__ == "__main__":
    root = tk.Tk()
    app = DroneCamApp(root)
    root.mainloop()
