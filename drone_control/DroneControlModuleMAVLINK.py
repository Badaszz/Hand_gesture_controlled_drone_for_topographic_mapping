#!/usr/bin/env python

#...........................................................................
# Control Drone with Keyboard Keys using pymavlink
# By Yusuf Solomon Olumide Badaszzzz
#...........................................................................

# Import Necessary Packages
from pymavlink import mavutil
import time, math
from sshkeyboard import listen_keyboard

class DroneKeyControl:
    def __init__(self,ip_address):
        
        self.ip_address = ip_address # drone ip address
        print("Connecting to Drone...")
        
        # try:
        #     # Connect to SITL or vehicle
        #     self.master = mavutil.mavlink_connection(self.ip_address)
        #     self.master.wait_heartbeat()
        #     self.armed = False # set Arm check to false
        #     print("Connected to Drone")
        # except Exception as e:
        #     print(f"Failed to connect to the drone at {self.ip_address}. Error: {e}")
        #     self.vehicle = None
        
        self.vehicle = None

    def takeoff(self, altitude):
        """

        This method sends commands to the vehicle take-off  from the ground to the desired
        altitude by using.

        Inputs:
            1.  altitude            -   TakeOff Altitude

        """
        # Set mode to GUIDED
        self.mode_g = 'GUIDED'
        self.mode_g_id = self.master.mode_mapping()[self.mode_g]
        self.master.set_mode(self.mode_g_id)
        
        # Arm the drone
        self.master.mav.command_long_send(self.master.target_system, 
                                          self.master.target_component, 
                                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                          0, 1, 0, 0, 0, 0, 0, 0)
        # Wait for drone to arm
        self.master.motors_armed_wait()
        self.armed = True # set arm check to true
        print("Armed!")
        
        # Take off to 10 meters
        self.master.mav.command_long_send(self.master.target_system, 
                                     self.master.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                     0, 0, 0, 0, 0, 0, 0, altitude)
        start_time = time.time()
        timeout = 60  # Timeout in seconds
        
        # Wait for the drone to reach the target altitude
        while True:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                current_altitude = msg.relative_alt / 1000.0  # Convert to meters
                print(f"Current altitude: {current_altitude} m")
                if current_altitude >= altitude - 1.5 :  
                    # Allow some tolerance for reaching the target altitude
                    print("Reached target altitude!")
                    break
            if time.time() - start_time > timeout:
                print("Timeout reached. Drone failed to reach the desired altitude.")
                break
            time.sleep(1)

    def Land(self):

        """

        This method will Land the drone.


        """
        # Land the drone
        self.master.mav.command_long_send(self.master.target_system, 
                                          self.master.target_component,
                                          mavutil.mavlink.MAV_CMD_NAV_LAND,
                                          0, 0, 0, 0, 0, 0, 0, 0)

        # Wait for the drone to land
        while True:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                altitude = msg.relative_alt / 1000.0  # Convert to meters
                print(f"Current altitude: {altitude} m")
                if altitude <= 0.5:  # Allow some tolerance for landing
                    print("Landed!")
                    break
            time.sleep(1) # Wait for a while before checking again
        time.sleep(2.5) # Wait for 2.5 seconds before disarming
        ## Disarm the drone after landing
        self.master.mav.command_long_send(self.master.target_system, 
                                          self.master.target_component, 
                                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
                                          0, 0, 0, 0, 0, 0, 0, 0)
        self.armed = False # set arm check to false
        


    def move(self, distance, direction):

        """

        This method will move the drone in a given direction.
        
        Inputs:
            1.  distance[0]          -  distance to move in metres . forward/backward positive forward, negative backward
            2.  distance[1]          -  distance to move in metres . right/left positive right, negative left
            3. distance[2]          -  distance to move in metres . up/down negative up, positive down
            4. direction            - 'x'/'y'/'z'

        """
        # Get current altitude
        msg_m = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
        if not msg_m:
            print("Failed to get current altitude.")
            return

        # current_alt = -msg_m.z  # Altitude above home (z is negative upward in NED)
        # # print(f"Current altitude: {current_alt:.2f} m")

        # # Define bounds
        # max_alt = 12  # meters
        # min_alt = 2.5  # meters

        # # Calculate target altitude
        # target_alt = current_alt - distance[2]  # because moving down is +ve z in NED
        # # print(f"Target altitude: {target_alt:.2f} m")

        # if target_alt > max_alt:
        #     print(f"Target altitude {target_alt:.2f}m exceeds max altitude. Aborting.")
        #     return
        # elif target_alt < min_alt:
        #     print(f"Target altitude {target_alt:.2f}m is below min altitude. Aborting.")
        #     return
        
        self.master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,
                                                                              self.master.target_system,
                                                                              self.master.target_component,
                                                                              mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                                                                              0b110111111000,  # ignore all but position
                                                                              distance[0], distance[1], distance[2],  # move X, Y, Z in meters
                                                                              0, 0, 0,  # no velocity
                                                                              0, 0, 0,  # no acceleration
                                                                              0, 0 # no yaw rate
                                                                              ))
        
        dist = distance[0] + distance[1] + distance[2]
        time.sleep(abs(dist+ (dist/2)))  # Wait for the drone to move
    
    def RTL(self, height):

        """

        This method will return the drone to the launch point.
        
        it takes height (in cm) as input, tis would be the RTL altitude

        """
        #set the RTL_ALT as the height
        self.master.mav.param_set_send(
                                        self.master.target_system,
                                        self.master.target_component,
                                        b'RTL_ALT',
                                        height,  # height in cm
                                        mavutil.mavlink.MAV_PARAM_TYPE_INT32
                                        )
        self.master.mav.command_long_send(
                                            self.master.target_system,        # target_system
                                            self.master.target_component,     # target_component
                                            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,  # command
                                            0,                           # confirmation
                                            0, 0, 0, 0, 0, 0, 0          # param1 ~ param7 (not used for RTL)
                                            )
        
        print("RTL command sent successfully!")
        
        # Wait until mode changes from RTL to something else
        while True:
            self.master.recv_match(type='HEARTBEAT', blocking=True)
            current_mode = mavutil.mode_string_v10(self.master.messages['HEARTBEAT'])
            print(f"Current mode: {current_mode}")

            if current_mode != "RTL":
                print(f"RTL complete. Mode changed to: {current_mode}")
                break

            time.sleep(1)

        print("Return to Home (RTL) command executed successfully.")
        
    def yaw(self, yaw_angle, yaw_rate, direction):

        """

        This method Yaws the drone (moves the drone about the Z axis) by 15 degrees 
        The drone only yaws relative to its current orientation.

        Inputs:
            1.  Yaw angle       -   15 degrees for each key press
            2.  Yaw rate      -   5 degrees per second
            3.  Direction       -   1 for clockwise, -1 for counter-clockwise

        """
        self.master.mav.command_long_send(self.master.target_system,
                                          self.master.target_component, 
                                          mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                                          0, # ignored
                                          yaw_angle, # Yaw angle in degrees
                                          yaw_rate, # Yaw rate in degrees per second
                                          direction, # Direction of yaw (1 for CW, -1 for CCW) 
                                          1, # Setting yaw mode to absolute
                                          0, 0, 0)
        print(f"Yawing {'Clockwise' if direction == 1 else 'Counter-Clockwise'} by {yaw_angle} degrees at {yaw_rate} degrees/second")
        time.sleep(int(yaw_angle / yaw_rate))  # Wait for the yaw to complete
        time.sleep(2)  # Add a small delay to avoid sending commands too quickly

    def control(self, value):

        """
        
        This method calls the respective methods based on the argument received.

            t             -       Take-Off
            l             -       Land
            r             -       RTL Mode
            i, k,         -       This will call the move() method  
            l, j          -       This will call the move() method
            w, s          -       This will call the move() method
            q,e           -       This will call the yaw() method q for CCW and e for CW

        Inputs:
            1.  value         -   ['t', 'b', 'r', 'q', 'e', 'i', 'k', 'l', 'j', 'w', 's']

        """

        allowed_keys = ['t', 'b', 'r', 'q', 'e', 'i', 'k', 'l', 'j', 'w', 's']

        if value in allowed_keys:
            if value == 't':
                if self.armed == False:
                    self.takeoff(altitude = 3.5) # Take off to 3.5 meters
                else:
                    print("Vehicle is in the air already")

            if value == 'b':
                if self.armed == False:
                    #check if vehicle is armed or not
                    print("Vehicle not Armed cannot Land")
                else:
                    print("Landing the drone")
                    self.Land()

            if value == 'r':
                if self.armed == False:
                    print("Vehicle not Armed Cannot RTL")
                else:
                    self.RTL(350)
                    
            if value == 'q':
                if self.armed == False:
                    print("Vehicle not Armed Cannot Yaw")
                else:
                    print("Yawing Counter-Clockwise")
                    self.yaw(15, 5, -1) # CCW Yaw
                    
            if value == 'e':
                if self.armed == False:
                    print("Vehicle not Armed Cannot Yaw")
                else:
                    print("Yawing Clockwise")
                    self.yaw(15, 5, 1)

            if value in allowed_keys[5:]:
                if self.armed == False:
                    print("Vehicle not Armed Cannot Move")
                else:
                    if value == 'i':
                        print("Moving Up")
                        try:
                            self.move([0, 0, -2], 'z')
                        except:
                            print("Error")
                            print("Aborting....")
                    elif value == 'k':
                        print("Moving Down")
                        self.move([0, 0, 2], 'z')
                    elif value == 'l':
                        print("Moving Right")
                        self.move([0, 3, 0], 'y')
                    elif value == 'j':
                        print("Moving Left")
                        self.move([0, -3, 0], 'y')
                    elif value == 'w':
                        print("Moving Forward")
                        self.move([3, 0, 0], 'x')
                    elif value == 's':
                        print("Moving Backward")
                        self.move([-3, 0, 0], 'x')

        else:
            print("Enter a valid Key!!!")

    def press(self, key):
        
        """
        
        This function prints the keybooard presses and calls the control()
        function.

        Inputs:
            1.  key         -   Pressed keyboard Key

        """
        
        print(f"'{key}' is pressed")

        # Sending Control Inputs
        self.control(value = key)

    def main(self):
        ## Main Function to run the program
        print("Press 't' to take off, 'b' to land, 'r' for RTL, 'q' for CCW Yaw, 'e' for CW Yaw")
        print("Press 'i' to move up, 'k' to move down, 'l' to move right, 'j' to move left")
        print("Press 'w' to move forward, 's' to move backward")
        # Listen for Keyboard presses
        listen_keyboard(on_press=self.press, until = 'esc')

if __name__ == "__main__":
    droneIP = 'udp:172.29.144.1:14550'
    drone = DroneKeyControl(ip_address = droneIP)
    drone.main()