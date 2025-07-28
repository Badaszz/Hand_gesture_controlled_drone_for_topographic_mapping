import socket
import time
import struct
import threading
import logging
import msvcrt  # For non-blocking keyboard input on Windows
import csv
from datetime import datetime

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[logging.FileHandler('msp_rc_log.txt'), logging.StreamHandler()]
)
logger = logging.getLogger(__name__)

# Global RC state
rc_data = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]  # Roll, Pitch, Throttle, Yaw, AUX1, AUX2, AUX3, AUX4
rc_lock = threading.Lock()
is_running = True
armed = False
pre_armed = False

def calculate_checksum(data):
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

def build_msp_message(code, payload=b''):
    length = len(payload)
    header = b'$M<'
    data = bytes([length, code]) + payload
    checksum = calculate_checksum(data)
    return header + data + bytes([checksum])

def parse_attitude_response(response):
    if not response or len(response) < 9 or response[3] != 6 or response[4] != 108:
        return None
    try:
        roll, pitch, yaw = struct.unpack('<hhh', response[5:11])
        return {'roll': roll / 10.0, 'pitch': pitch / 10.0, 'yaw': yaw}
    except Exception as e:
        # logger.error(f"Error parsing MSP_ATTITUDE response: {e}")
        return None

def send_msp_command(sock, cmd_name, cmd_code, payload=b'', retries=3, timeout=2):
    import csv
    import datetime
    for attempt in range(retries):
        try:
            message = build_msp_message(cmd_code, payload)
            logger.info(f"Sending {cmd_name} (Code {cmd_code}, attempt {attempt + 1}): {message}")
            sock.sendall(message)
            sock.settimeout(timeout)
            response = sock.recv(1024)
            logger.info(f"{cmd_name} response: {response}")
            if cmd_name == 'MSP_ATTITUDE':
                parsed = parse_attitude_response(response)
                if parsed:
                    print(f"{cmd_name} parsed: Roll={parsed['roll']:.1f}°, Pitch={parsed['pitch']:.1f}°, Yaw={parsed['yaw']:.1f}°")
                    with open('drone_data.csv', 'a', newline='') as csv_file:
                        csv_writer = csv.writer(csv_file)
                        csv_writer.writerow([datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'), parsed['roll'], parsed['pitch'], parsed['yaw'], 0.0])
                return parsed
            elif cmd_name == 'MSP_ALTITUDE':
                parsed = parse_altitude_response(response)
                if parsed:
                    print(f"{cmd_name} parsed: Altitude={parsed['altitude']:.2f}m")
                    with open('drone_data.csv', 'a', newline='') as csv_file:
                        csv_writer = csv.writer(csv_file)
                        csv_writer.writerow([datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'), 0.0, 0.0, 0.0, parsed['altitude']])
                return parsed
            return response
        except socket.timeout:
            logger.warning(f"No response for {cmd_name} (Code {cmd_code})")
        except Exception as e:
            logger.error(f"Error sending {cmd_name} (Code {cmd_code}): {e}")
    return None

def parse_altitude_response(response):
    if not response or len(response) < 9 or response[3] != 6 or response[4] != 109:
        return None
    try:
        altitude_cm, vario = struct.unpack('<ih', response[5:11])
        return {'altitude': altitude_cm / 100.0}
    except Exception as e:
        logger.error(f"Error parsing MSP_ALTITUDE response: {e}")
        return None


def rc_transmitter(sock):
    """Continuously send MSP_SET_RAW_RC at 20Hz"""
    global rc_data, is_running
    while is_running:
        with rc_lock:
            payload = struct.pack('<HHHHHHHH', *rc_data)
        send_msp_command(sock, 'MSP_SET_RAW_RC', 200, payload)
        time.sleep(0.05)  # 20Hz

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    sock.connect(('192.168.4.1', 2323))
    print("TCP connection successful!")
    # logger.info("Connected to 192.168.4.1:2323")
    
    # Verify connection
    response = send_msp_command(sock, 'MSP_API_VERSION', 1)
    if response:
        print(f"MSP_API_VERSION response: {response}")
    
    # Start RC transmitter thread
    rc_thread = threading.Thread(target=rc_transmitter, args=(sock,), daemon=True)
    rc_thread.start()
    
    print("Press Enter to pre-arm, Enter again to arm, Enter to disarm (press 'q' to quit)...")
    last_attitude_time = time.time()
    last_attitude = {'roll': 0, 'pitch': 0, 'yaw': 0}
    up = 1000
    while True:
        # Handle Enter key for pre-arm/arm/disarm
        if msvcrt.kbhit():
            key = msvcrt.getch()
            if key == b'\r':  # Enter key
                with rc_lock:
                    if not pre_armed:
                        print("Pre-arming (AUX2 high)...")
                        rc_data[5] = 1300  # Channel 6 (AUX2) high
                        pre_armed = True
                    elif pre_armed and not armed:
                        print("Arming (AUX1 high)...")
                        rc_data[4] = 1300  # Channel 5 (AUX1) high
                        armed = True
                    elif armed:
                        print("Disarming (AUX1, AUX2 low)...")
                        rc_data[4] = 1000  # Channel 5 (AUX1) low
                        rc_data[5] = 1000  # Channel 6 (AUX2) low
                        armed = False
                        pre_armed = False
            elif key == b'u':
                up += 25
                rc_data[2] = up
                print(up)
            elif key == b'j':
                up -= 25
                rc_data[2] = up
                print(up)
            elif key == b'q':
                rc_data[4] = 1000  # Channel 5 (AUX1) low
                rc_data[5] = 1000  # Channel 6 (AUX2) low
                break
        
        # Poll attitude every 0.1s
        if time.time() - last_attitude_time >= 0.1:
            attitude = send_msp_command(sock, 'MSP_ATTITUDE', 108)
            altitude = send_msp_command(sock, 'MSP_ALTITUDE', 108)
            if attitude:
                last_attitude = attitude
            last_attitude_time = time.time()
        
        # Print RC and attitude status
        #with rc_lock:
            #print(f"RC: Roll={rc_data[0]}, Pitch={rc_data[1]}, Throttle={rc_data[2]}, AUX1={rc_data[4]}, AUX2={rc_data[5]} | "
            #      f"Attitude: Roll={last_attitude['roll']:.1f}°, Pitch={last_attitude['pitch']:.1f}°, Yaw={last_attitude['yaw']:.1f}°")

        time.sleep(0.05)  # 20Hz main loop
    
    is_running = False

except socket.error as e:
    print(f"TCP connection error: {e}")
    logger.error(f"TCP connection error: {e}")
except KeyboardInterrupt:
    print("Stopped by user")
    logger.info("Stopped by user")
except Exception as e:
    print(f"Unexpected error: {e}")
    logger.error(f"Unexpected error: {e}")
finally:
    is_running = False
    if 'sock' in locals():
        sock.close()
        print("Socket closed")
    logger.info("Program terminated")