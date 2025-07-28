import socket
import time
import struct
import threading
import logging

class DroneKeyControl:
    def __init__(self, ip_address='192.168.4.1', port=2323):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5)
        self.address = (ip_address, port)

        self.rc_data = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
        self.rc_lock = threading.Lock()
        self.is_running = True
        self.armed = False
        self.pre_armed = False
        self.connect()
        threading.Thread(target=self.rc_transmitter, daemon=True).start()

    def connect(self):
        try:
            self.sock.connect(self.address)
            print("TCP connection successful!")
        except Exception as e:
            print(f"Connection failed: {e}")

    def calculate_checksum(self, data):
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def build_msp_message(self, code, payload=b''):
        length = len(payload)
        header = b'$M<'
        data = bytes([length, code]) + payload
        checksum = self.calculate_checksum(data)
        return header + data + bytes([checksum])

    def send_msp_command(self, cmd_name, cmd_code, payload=b''):
        try:
            msg = self.build_msp_message(cmd_code, payload)
            self.sock.sendall(msg)
        except Exception as e:
            print(f"Error sending {cmd_name}: {e}")

    def rc_transmitter(self):
        while self.is_running:
            with self.rc_lock:
                payload = struct.pack('<HHHHHHHH', *self.rc_data)
            self.send_msp_command('MSP_SET_RAW_RC', 200, payload)
            time.sleep(0.05)

    # Drone Command Methods
    def pre_arm(self):
        with self.rc_lock:
            self.rc_data[5] = 1300
        self.pre_armed = True

    def arm(self):
        with self.rc_lock:
            self.rc_data[4] = 1300
        self.armed = True

    def disarm(self):
        with self.rc_lock:
            self.rc_data[4] = 1000
            time.sleep(1)
            self.rc_data[5] = 1000
        self.armed = False
        self.pre_armed = False

    def increase_throttle(self, amount=100):
        with self.rc_lock:
            self.rc_data[2] = min(self.rc_data[2] + amount, 2000)

    def decrease_throttle(self, amount=100):
        with self.rc_lock:
            self.rc_data[2] = max(self.rc_data[2] - amount, 1000)

    def move_forward(self, amount=100):
        with self.rc_lock:
            self.rc_data[1] = 1500 + amount

    def move_backward(self, amount=100):
        with self.rc_lock:
            self.rc_data[1] = 1500 - amount

    def move_left(self, amount=100):
        with self.rc_lock:
            self.rc_data[0] = 1500 - amount

    def move_right(self, amount=100):
        with self.rc_lock:
            self.rc_data[0] = 1500 + amount

    def yaw_left(self, amount=100):
        with self.rc_lock:
            self.rc_data[3] = 1500 - amount

    def yaw_right(self, amount=100):
        with self.rc_lock:
            self.rc_data[3] = 1500 + amount

    def hover(self):
        with self.rc_lock:
            self.rc_data[0] = 1500
            self.rc_data[1] = 1500
            self.rc_data[2] = 1500
            self.rc_data[3] = 1500

    def square_movement(self):
        print("Executing square movement...")
        self.pre_arm()
        time.sleep(1)
        self.arm()
        time.sleep(3)
        self.disarm()
        print('done')
        # self.move_forward(200); time.sleep(3); self.hover(); time.sleep(3)
        # self.yaw_right(200); time.sleep(3); self.hover(); time.sleep(3)
        # self.move_right(200); time.sleep(3); self.hover(); time.sleep(3)
        # self.move_backward(200); time.sleep(3); self.hover(); time.sleep(3)
        # self.move_left(200); time.sleep(3); self.hover(); time.sleep(3)
        # self.hover(); time.sleep(1)

    def altitude_hold(self):
        print("Holding altitude...")
        with self.rc_lock:
            self.rc_data[6] = 1500
        
    def press(self, key):
        control_dict = {
            'TAKEOFF': 't',
            'LAND': 'b',
            'MAP_AREA': 'r',
            'INC': 'i',
            'DEC': 'k',
            'RIGHT': 'l',
            'LEFT': 'j',
            'YAW_LEFT': 'q',
            'YAW_RIGHT': 'e',
            'FORWARD': 'w',
            'BACKWARD': 's'
        }
        if key in control_dict.values():
            if key == 't':
                print("Taking off...")
                self.pre_arm()
                time.sleep(1)
                self.arm()
                time.sleep(1)
                self.increase_throttle(200)
                time.sleep(1)
                self.increase_throttle(200)
                time.sleep(1)
                self.increase_throttle(100)
                time.sleep(1)
                self.altitude_hold()
                time.sleep(1)
            elif key == 'b':
                print("Landing...")
                self.decrease_throttle(200)
                time.sleep(1)
                self.decrease_throttle(200)
                time.sleep(1)
                self.decrease_throttle(100)
                time.sleep(1)
                self.disarm()
            elif key == 'r':
                print("Mapping area...")
                self.square_movement()
                time.sleep(1)
            elif key == 'i':
                print("Increasing throttle...")
                self.increase_throttle(200)
                time.sleep(2)
                self.hover()
                time.sleep(1)
            elif key == 'k':
                print("Decreasing throttle...")
                self.decrease_throttle(200)
                time.sleep(2)
                self.hover()
                time.sleep(1)
            elif key == 'l':
                print("Moving right...")
                self.move_right(200)
                time.sleep(2)
                self.hover()
                time.sleep(1)
            elif key == 'j':
                print("Moving left...")
                self.move_left(200)
                time.sleep(2)
                self.hover()
                time.sleep(1)
            elif key == 'q':
                print("Yawing left...")
                self.yaw_left(200)
                time.sleep(2)
                self.hover()
                time.sleep(1)
            elif key == 'e':
                print("Yawing right...")
                self.yaw_right(200)
                time.sleep(2)
                self.hover()
                time.sleep(1)
            elif key == 'w':
                print("Moving forward...")
                self.move_forward(200)
                time.sleep(2)
                self.hover()
                time.sleep(1)
            elif key == 's':
                print("Moving backward...")
                self.move_backward(200)
                time.sleep(2)
                self.hover()
                time.sleep(1)
        else:
            print(f"Unknown key: {key}")
    
    def main(self):
        print("Drone control started. Press 't' to take off, 'b' to land, or other keys for movement.")
        while self.is_running:
            key = input("Enter command: ")
            if key == 'c':
                print("Exiting drone control.")
                self.is_running = False
                break
            else:
                self.press(key)

if __name__ == "__main__":
    drone = DroneKeyControl()
    drone.connect()
    drone.pre_arm()
    time.sleep(1)
    drone.arm()
    print("Drone is armed and ready.")
    time.sleep(3)
    drone.disarm()
    drone.main()