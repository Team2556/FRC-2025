# create a commands2 subsyteem for the ultrasonic sensor 
# the sensor input will be supplied by a RIOduino
# Example code for a standalone python program is below
#import serial
# import time

# # Replace 'COM3' with your Arduino's serial port; find it in the Arduino IDE
# arduino_port = serial.Serial(port='COM4', baudrate=9600, timeout=.1)
# time.sleep(2) # wait a bit for the connection to establish

# while True:
# 	try:
# 		if arduino_port.in_waiting > 0:
# 			data = arduino_port.readline().decode('utf-8').rstrip()
# 			print(data)
# 	except serial.SerialException:
# 		print("Serial connection lost.")
# 		break
# 	except UnicodeDecodeError:
# 		print("Could not decode data")
# 	time.sleep(0.1)


import wpilib
import commands2
from wpilib import AnalogInput, SmartDashboard
from constants import UltrasonicConstants



class Ultrasonic(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.frontLeft_ultrasonic = AnalogInput(UltrasonicConstants.frontLeft)
        self.frontRight_ultrasonic = AnalogInput(UltrasonicConstants.frontRight)
        self.backLeft_ultrasonic = AnalogInput(UltrasonicConstants.backLeft)
        self.backRight_ultrasonic = AnalogInput(UltrasonicConstants.backRight)

        
        # '/dev/ttyUSB0'
    #     self.arduino_port = serial.Serial(port='/dev/bus/usb/001', baudrate=9600, timeout=.1)
    #     time.sleep(2) # wait a bit for the connection to establish

    # def get_distance(self):
    #     try:
    #         if self.arduino_port.in_waiting > 0:
    #             data = self.arduino_port.readline().decode('utf-8').rstrip()
    #             return data
    #     except serial.SerialException:
    #         print("Serial connection lost.")
    #     except UnicodeDecodeError:
    #         print("Could not decode data")
    #     return None
    
    def periodic(self):
        frontLeft_distance = self.frontLeft_ultrasonic.getVoltage() * 1000 *.042
        frontLeft_distance_valid = frontLeft_distance >12.5 and frontLeft_distance < 250
        SmartDashboard.putBoolean("Front Left Ultrasonic Distance Valid", frontLeft_distance_valid)
        SmartDashboard.putNumber("Front Left Ultrasonic Distance", float(frontLeft_distance))
        frontRight_distance = self.frontRight_ultrasonic.getVoltage() * 1000 *.042
        frontRight_distance_valid = frontRight_distance >12.5 and frontRight_distance < 250 
        SmartDashboard.putBoolean("Front Right Ultrasonic Distance Valid", frontRight_distance_valid)
        SmartDashboard.putNumber("Front Right Ultrasonic Distance", float(frontRight_distance))
        backLeft_distance = self.backLeft_ultrasonic.getVoltage() * 1000 *.042
        backLeft_distance_valid = backLeft_distance >12.5 and backLeft_distance < 250
        SmartDashboard.putBoolean("Back Left Ultrasonic Distance Valid", backLeft_distance_valid)
        SmartDashboard.putNumber("Back Left Ultrasonic Distance", float(backLeft_distance))
        backRight_distance = self.backRight_ultrasonic.getVoltage() * 1000 *.042
        backRight_distance_valid = backRight_distance >12.5 and backRight_distance < 250
        SmartDashboard.putBoolean("Back Right Ultrasonic Distance Valid", backRight_distance_valid)
        SmartDashboard.putNumber("Back Right Ultrasonic Distance", float(backRight_distance))

     

