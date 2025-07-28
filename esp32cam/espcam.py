import websocket
import threading
import os
import time

class ESP32CamClient:
    def __init__(self, ip="192.168.8.247", port=81):
        self.url = f"ws://{ip}:{port}"
        self.ws = None
        self.connected = False
        self.image_index = 0
        self.download_dir = "downloaded_images"
        os.makedirs(self.download_dir, exist_ok=True)

    def on_open(self, ws):
        print("[✓] Connected to ESP32-CAM WebSocket")
        self.connected = True

    def on_close(self, ws, close_status_code, close_msg):
        print("[✖] Disconnected")
        self.connected = False

    def on_message(self, ws, message):
        if isinstance(message, bytes):
            filename = os.path.join(self.download_dir, f"img_{self.image_index}.jpg")
            with open(filename, 'wb') as f:
                f.write(message)
            print(f"[💾] Received image saved as {filename}")
            self.image_index += 1
        else:
            print(f"[ESP32]: {message}")

    def connect(self):
        self.ws = websocket.WebSocketApp(self.url,
                                         on_open=self.on_open,
                                         on_close=self.on_close,
                                         on_message=self.on_message)
        thread = threading.Thread(target=self.ws.run_forever)
        thread.daemon = True
        thread.start()
        time.sleep(2)

    def send_command(self, cmd):
        if self.connected:
            self.ws.send(cmd)
            print(f"[→] Sent command: {cmd}")
        else:
            print("[!] Not connected")

    def start_capture(self):
        self.send_command("start")

    def stop_capture(self):
        self.send_command("stop")

    def download_images(self):
        self.image_index = 0  # Reset index to avoid filename clashes
        self.send_command("download")

    def clear_storage(self):
        self.send_command("clear")

    def close(self):
        self.ws.close()
    
    def run_flight_sequence(self, duration_sec):
        self.start_capture()
        time.sleep(duration_sec)
        self.stop_capture()
        self.download_images()
        self.clear_storage()
        print("[✓] Flight sequence completed")

if __name__ == "__main__":
    esp = ESP32CamClient("192.168.4.3")  # Replace with your ESP32-CAM IP
    esp.connect()

    print("\nCommands:")
    print("1 = start capture")
    print("2 = stop capture")
    print("3 = download images")
    print("4 = clear storage")
    print("0 = exit\n")

    try:
        while True:
            choice = input("Enter command: ")
            if choice == '1':
                esp.start_capture()
            elif choice == '2':
                esp.stop_capture()
            elif choice == '3':
                esp.download_images()
            elif choice == '4':
                esp.clear_storage()
            elif choice == '0':
                esp.close()
                break
            else:
                print("[!] Invalid command")
    except KeyboardInterrupt:
        esp.close()
        print("\n[✖] Program exited")
