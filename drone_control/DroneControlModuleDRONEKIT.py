#!/usr/bin/env python

#...........................................................................
# Control Drone with Keyboard
# Revised By Yusuf Solomon Olumide
# Github: Link to be added


# Author:  Saiffullah Sabir Mohamed
# Github:  https://github.com/TechnicalVillager
# Website: http://technicalvillager.github.io/
# Source: https://github.com/TechnicalVillager/control-drone-with-keyboard/
#...........................................................................

# Import Necessary Packages
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time, math
from sshkeyboard import listen_keyboard

class DroneKeyControl:
    def __init__(self,ip_address):
        # Connecting the Vehicle
        self.ip_address = ip_address
        print("Connecting to Drone...")
        try:
            self.vehicle = connect(self.ip_address, wait_ready=True, baud=115200, timeout=60)
        except Exception as e:
            print(f"Failed to connect to the drone at {self.ip_address}. Error: {e}")
            self.vehicle = None
        print("Connected to Drone")

    def basic_takeoff(self, altitude):
        """

        This method sends commands to the vehicle take-off  from the ground to the desired
        altitude by using dronekit's simple_takeoff() function.

        Inputs:
            1.  altitude            -   TakeOff Altitude

        """
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        self.vehicle.simple_takeoff(altitude)
        time.sleep(2)
        print("Drone Armed")
        start_time = time.time()
        timeout = 60  # Timeout in seconds

        while True:
            print("Drone has Reached Height = ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= (altitude - 1.5):
                break
            if time.time() - start_time > timeout:
                print("Timeout reached. Drone failed to reach the desired altitude.")
                break
            time.sleep(1)
            print("Drone has Reached Height = ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= (altitude - 1.5):
                break
            time.sleep(1)


    def change_mode(self, mode):

        """

        This method will change the mode of the Vehicle.

        Inputs:
            1.  mode            -   Vehicle's Mode
        self.vehicle.mode = VehicleMode(mode)
        time.sleep(2)  # Allow time for the mode change to take effect
        if self.vehicle.mode.name == mode:
            print(f"Vehicle Mode Changed to {mode}")
        else:
            print(f"Failed to change mode to {mode}. Current mode: {self.vehicle.mode.name}")
            1.  GUIDED
            2.  LAND
            3.  RTL 
        """
        self.vehicle.mode = VehicleMode(mode)
        print(f"Vehicle Mode Changed to {mode}")

    def send_to(self, latitude, longitude, altitude):

        """

        This method will send the drone to desired location, when the 
        vehicle is in GUIDED mode.
        
        Inputs:
            1.  latitude            -   Destination location's Latitude
            2.  longitude           -   Destination location's Longitude
            3.  altitude            -   Vehicle's flight Altitude

        """

        if self.vehicle.mode.name == "GUIDED":
            location = LocationGlobalRelative(latitude, longitude, float(altitude))
            self.vehicle.simple_goto(location)
            time.sleep(1)

    def change_alt(self, step):

        """
        
        This method will increase or decrease the altitude
        of the vehicle based on the input.

        Inputs:
            1.  step            -   Increase 5 meters of altitude from 
                                    current altitude when INC is passed as argument.

                                -   Decrease 5 meters of altitude from 
                                    current altitude when DEC is passed as argument.

        """

        actual_altitude = int(self.vehicle.location.global_relative_frame.alt)
        changed_altitude = [(actual_altitude + 5), (actual_altitude - 5)]

        if step == "INC":
            if changed_altitude[0] <= 20:
                self.send_to(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon, changed_altitude[0])
            else:
                print("Vehicle Reached Maximum Altitude!!!")

        if step == "DEC":
            if changed_altitude[1] >= 5:
                self.send_to(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon, changed_altitude[1])
            else:
                print("Vehicle Reached Minimum Altitude!!!")


    def destination_location(self, homeLattitude, homeLongitude, distance, bearing):

        """

        This method returns the latitude and longitude of the
        destination location, when distance and bearing is provided.

        Inputs:
            1.  homeLattitude       -   Home or Current Location's  Latitude
            2.  homeLongitude       -   Home or Current Location's  Longitude
            3.  distance            -   Distance from the home location
            4.  bearing             -   Bearing angle from the home location

        """

        #Radius of earth in metres
        R = 6371e3

        rlat1 = homeLattitude * (math.pi/180) 
        rlon1 = homeLongitude * (math.pi/180)

        d = distance

        #Converting bearing to radians
        bearing = bearing * (math.pi/180)

        rlat2 = math.asin((math.sin(rlat1) * math.cos(d/R)) + (math.cos(rlat1) * math.sin(d/R) * math.cos(bearing)))
        rlon2 = rlon1 + math.atan2((math.sin(bearing) * math.sin(d/R) * math.cos(rlat1)) , (math.cos(d/R) - (math.sin(rlat1) * math.sin(rlat2))))

        #Converting to degrees
        rlat2 = rlat2 * (180/math.pi) 
        rlon2 = rlon2 * (180/math.pi)

        # Lat and Long as an Array
        location = [rlat2, rlon2]

        return location

    def control(self, value):

        """
        
        This method calls the respective methods based on the argument received.

            t             -       Take-Off
            l             -       Land
            g             -       Guided Mode
            r             -       RTL Mode
            up, down,
            right, left   -       This will call the navigation() function 

        Inputs:
            1.  value         -   ['space', 'tab', 't', 'l', 'g', 'r', 'up', 'down', 'right', 'left']

        """

        allowed_keys = ['space', 'tab', 't', 'l', 'g', 'r', 'up', 'down', 'right', 'left']

        if value in allowed_keys:

            if value == 'space':
                if not self.vehicle.armed:
                    print("Vehicle not Armed Canot Increase Altitude")
                else:
                    self.change_alt(step = "INC")

            if value == 'tab':
                if not self.vehicle.armed:
                    print("Vehicle not Armed Canot Decrease Altitude")
                else:
                    self.change_alt(step = "DEC")

            if value == 't':
                if int(self.vehicle.location.global_relative_frame.alt) <= 5:
                    self.basic_takeoff(altitude = 5)

            if value == 'l':
                if self.vehicle.armed == False:
                    #check if vehicle is armed or not
                    print("Vehicle not Armed cannot Land")
                else:
                    self.change_mode(mode = "LAND")

            if value == 'g':
                self.change_mode(mode = "GUIDED")

            if value == 'r':
                if self.vehicle.armed == False:
                    print("Vehicle not Armed Canot RTL")
                else:
                    self.change_mode(mode = "RTL")

            if value in allowed_keys[-4:]:
                if self.vehicle.armed == False:
                    print("Vehicle not Armed Cannot Move")
                else:
                    self.navigation(value = value)

        else:
            print("Enter a valid Key!!!")

    def navigation(self, value):

        """
        
        This method moves the vehicle to front, back, right, left
        based on the input argument.

            UP       -   Moves the Vehicle to Forward
            DOWN     -   Moves the Vehicle to Backward
            RIGHT    -   Moves the Vehicle to Right
            LEFT     -   Moves the Vehicle to Left

        Inputs:
            1.  value         -   [right, left, up, down]

        """

        # Vehicle Location
        angle = int(self.vehicle.heading)
        loc   = (self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon, self.vehicle.location.global_relative_frame.alt)

        # Default Distance in meters
        default_distance = 5

        if value == 'up':
            front = angle + 0
            new_loc = self.destination_location(homeLattitude = loc[0], homeLongitude = loc[1], distance = default_distance, bearing = front)
            self.send_to(new_loc[0], new_loc[1], loc[2])

        if value == 'down':
            back = angle + 180
            new_loc = self.destination_location(homeLattitude = loc[0], homeLongitude = loc[1], distance = default_distance, bearing = back)
            self.send_to(new_loc[0], new_loc[1], loc[2])

        if value == 'right':
            right = angle + 90
            new_loc = self.destination_location(homeLattitude = loc[0], homeLongitude = loc[1], distance = default_distance, bearing = right)
            self.send_to(new_loc[0], new_loc[1], loc[2])

        if value == 'left':
            left = angle -90
            new_loc = self.destination_location(homeLattitude = loc[0], homeLongitude = loc[1], distance = default_distance, bearing = left)
            self.send_to(new_loc[0], new_loc[1], loc[2])

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
        ## Main Function
        # Declaring Vehicle as global variable

        # Setting the Heading angle constant throughout flight
        if self.vehicle.parameters['WP_YAW_BEHAVIOR'] != 0:
            self.vehicle.parameters['WP_YAW_BEHAVIOR'] = 0
            print("Changed the Vehicle's WP_YAW_BEHAVIOR parameter")

        # Listen Keyboard Keys
        listen_keyboard(on_press=self.press)

if __name__ == "__main__":
    droneIP = 'udp:172.29.144.1:14550'
    drone = DroneKeyControl(ip_address = droneIP)
    drone.main()