# Python Library for the SwissCHEESE Hat made by CARobotics
# Developed by Andrew Hua
# Credits go to the respective owners of the imported libraries

# These are the libraries that should be installed in order to use this library
# There MAY be some redundancies in the libraries here
# sudo pip3 install adafruit-gpio
# sudo pip3 install adafruit-mcp3008
# sudo pip3 install board
# sudo pip3 install adafruit-circuitpython-pca9685

# Additionally, in order for the Pi to work with this library, it must have the following features enabled
# SSH Enabled
# SPI Enabled
# I2C Enabled
# Newer version of the Raspbian (9 or 10)

# TO-DO:
# - Implement Error and Exception Handling
# - Fix Ultra Sonic Sensing
# - Implement proper Getters & Setters for different variables
# - Implement unique functions for certain modules
# - Implement a setup.py or something to handle the installation of the libraries / dependencies
# - Implement possible method overloading?
# - Figure out how to program the EEPROM

# Math module imported to use ceiling / floor functions for proper quantification of analog values from the sensor
import math

# Time module for time functions
import time

# SPI Modules for the MCP3008 IC
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008 as MCP

# I2C Modules for the PCA9685 IC
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
import busio

# File Checking
import os

########
# HAT INFORMATION
########

# Functions to print information associated with the EEPROM
# EEPROM Setup was made through this tutorial:
# https://hackaday.io/project/11222-raspberry-pi-zero-breakout/log/42353-setting-up-hat-eeprom

def printInfo():
    basepath = '/proc/device-tree/hat/'

    with os.scandir(basepath) as entries:
        for entry in entries:
            if entry.is_file():
                with open(entry, 'r',) as f:
                    print(entry.name, ': ', f.read())


def printVendor():
    basepath = '/proc/device-tree/hat/vendor'

    with open (basepath, 'r') as f:
        print('Vendor: ', f.read())

def printUUID():
    basepath = '/proc/device-tree/hat/uuid'

    with open (basepath, 'r') as f:
        print('UUID: ', f.read())

def printProdVer():
    basepath = '/proc/device-tree/hat/product_ver'

    with open (basepath, 'r') as f:
        print('Product Version: ', f.read())

def printHatName():
    basepath = '/proc/device-tree/hat/name'

    with open (basepath, 'r') as f:
        print('Name: ', f.read())
def printProduct():
    basepath = '/proc/device-tree/hat/product'

    with open (basepath, 'r') as f:
        print('Product: ', f.read())

def printProductID():
    basepath = '/proc/device-tree/hat/product_id'

    with open (basepath, 'r') as f:
        print('Product ID: ', f.read())

#########
# TIME FUNCTIONS
#########

# The delay function will mimic the delay function which exists in an Arduino, although it is entirely feasible
# to also simply use time.sleep() if the user wishes to associate delays in larger units of time (i.e. seconds)

def delay(milliseconds):
    time.sleep(milliseconds / 1000)
    #print('time delay of  {0} millisecond(s)'.format(milliseconds/1000))

# The delayMicro function will mimic the same function in Arduino, with the purpose of being even shorter
# amount of time, intended for unique circumstances and hardware

def delayMicro(microseconds):
    time.sleep(microseconds/1000000)
    #print('time delay of {0} microsecond(s)'.format(microseconds/1000000))

# There may be plans to include digital input to complement the analog input

#######
# INPUT (MCP3008)
#######

# Input is directly involved from the input port on the hat and then goes through the Analog-to-Digital Converter,
# which is the MCP3008 IC. The input channels are respective to the input ports on the SwissCHEESE Hat, from A0 to
# A7. The output is read through SPI or Serial Peripheral Interface. A lot of the code was inherited from Adafruit's
# open source repository for the MCP3008 module.

# Hardware SPI configurations for the Pi
SPI_PORT = 0
SPI_DEVICE = 0
mcp = MCP.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

# Parent input class for all general peripheral input devices - limited functions
class inputPort:

    # Initialization of the port number being used
    portNum = -1

    # Maximum value of the analog values (actually is 1023, but intended purpose is for function use)
    analogMax = 1024

    # Constructor for the inputPort
    def __init__(self, portNum):
            # The only valid ports accepted will be from 0 to 7 (referred to as J1 to J8)
            if (portNum > -1 and portNum < 8) :
                self.portNum = portNum
            else:
                raise ValueError("Must pick a valid port number from 0-7!")

    # Function to print the active value that is being read by the output IN ANALOG terms
    def printValue(self):
        print('Currently reading in a value of: ', mcp.read_adc(self.portNum))

    # Function responsible for simply returning the current value of the input (useful for conditionals)
    def currValue(self):
        return mcp.read_adc(self.portNum)

    # Function for returning current analog value to a smaller scaled number (power of 2) to reduce 'noise' variation
    # in the value of the input
    # Magnitude refers to how much it is divided by and what it should round to (uses floor function)
    # Recursively iterates through the different magnitudes of 2
    # ONLY accepts values that are powers of 2, up to 1024
    def currValueRounded(self, magnitude):
        if magnitude == 1:
            self.analogMax = 1024
            return self.currValue()
        elif magnitude == self.analogMax:
            return math.floor(self.currValue() / self.analogMax)
        else:
            self.analogMax = self.analogMax / 2
            return self.currValueRounded(magnitude)

    # Function responsible for returning the current port of the referenced inputPort object
    def currPort(self):
        return self.portNum

# Input class for exclusively the button
# Inherits the inputPort
class button(inputPort):

    # Constructor for the button
    def __init__(self, portNum):
        super().__init__(portNum)

    # Function responsible for detecting whether a button input has been received
    # Can be used as a conditional statement to gate whether a button input has been received in the code
    def buttonPress(self):
        # Unsure about the correct analog value threshold to use as typically input goes above 1000
        if mcp.read_adc(self.portNum) > 800:
            #print('Button has been pressed')
            return True
        else:
            #print('Button has not been pressed')
            return False

    # Function responsible for halting the code until the specified button has been pressed
    def untilButtonPress(self):
        while mcp.read.adc(self.portNum) < 800:
            # Does nothing until the button has been pressed
            pass

# Input class for exclusively the potentiometer
# Inherits the inputPort
class potentiometer(inputPort):

    # Constructor for the potentiometer
    def __init__(self, portNum):
        super().__init__(portNum)

    # Function responsible for 'sizing' down the analog range to whatever specified range that the user wishes to have
    # It takes the current analog values of 0-1023, divides it by 1024, and multiplies it by the range
    # it has the correct ratio. It is then goes through the ceiling function to turn it from a float to an int
    def map(self, range):
        return math.ceil((mcp.read_adc(self.portNum)* range)/1024)

# Input class for exclusively the IR sensor
# Inherits the input port
class irsensor(inputPort):

    # Constructor for the IR sensor
    def __init__(self, portNum):
        super().__init__(portNum)

# Input class for exclusively the LDR sensor
# Inherits the input port
class ldrsensor(inputPort):

    # Constructor for the LDR Sensor
    def __init__(self, portNum):
        super().__init__(portNum)

#######
# OUTPUT (PCA9685)
#######

# Output is directly involved with the output ports on the SwissCHEESE Hat. The output uses the SCL and SDA ports on
# the Pi (Serial Clock and Serial Data respectively) to send specific signals through one of the 16 channels on the
# PCA9685, which then sends the PWM (Pulse Width Modulation) signals to the respective output ports, whether it be
# actual output ports (J9 to J16) or the motor ports. The interface used is I2C or Inter-Integrated Circuit)
# Allegedly, the circuit has a 12-bit resolution, but is able to produce up to 16-bit resolution
# As a result, when setting PWM, your mileage may vary on the degree of success with the devices

# Hardware specifications for the I2C
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)

# Frequency associated with the PWM
# Too high of a frequency causes issues, so we settled for 60, which allows functionality
# for both LEDs and the motor drivers.
# Currently trying to determine the most optimal frequency for performance
pca.frequency = 60

# Parent output class for almost all general peripheral output devices - limited functions
class outputPort:

    # Value for the current PWM duty-cycle that is active on the port
    # Used for tracking and tracing primarily
    pwm = 0

    # Output max for 12-bit resolution and 16-bit resolution (both are one greater than actual max)
    # The 'max' allegedly is 12 bits, however, it is still possible to output a max of 16-bits
    softMax = 4096
    hardMax = 65535

    # Constructor for the outputPort
    def __init__(self, portNum):
        # The only valid ports accepted will be from 0 to 7 (referred to as J9 to J16)
        if (portNum > -1 and portNum < 8):

            # Mapping for the output ports to the correct PWM designation
            mapping = {0: 6,
                       1: 7,
                       2: 8,
                       3: 9,
                       4: 10,
                       5: 11,
                       6: 12,
                       7: 13}

            self.portNum = mapping[portNum]
            self.channel = pca.channels[self.portNum]

            # print("PWM:", self.portNum)
        else:
            raise ValueError("Must be a valid port from 0-7!")

    # Function responsible for a brute-force setting of the PWM to the device
    # Only accepts positive values of PWM by the PCA9685 (0 to 0xffff)
    # Be warned as not all devices may accept PWM values beyond 12-bits (0-4095)
    def setPWM(self, pwm):
        if pwm >= 0 and pwm <= 0xffff:
            self.pwm = pwm
            self.channel.duty_cycle = pwm
        else:
            raise ValueError("PWM duty cycle must be greater than 0 and less than 0xffff!")

    # Function responsible to return the value of the current duty-cycle active on the port
    def currPWM(self):
        return self.pwm

    # Function responsible to return the current port of the referenced outputPort object
    def currPort(self):
        return self.portNum

    # Function intended to clear the duty-cycle being sent through
    def clear(self):
        self.pwm = 0
        self.channel.duty_cycle = 0

# Function responsible for clearing all PWM channels
def fullClear():
    for i in range(16):
        pca.channels[i].duty_cycle = 0

# Function responsible for returning the current value of the duty-cycle of a PWM channel
# Only valid for integers 0 to 15 inclusive
def currentDuty(channel):
    if channel >= 0 and channel <= 15:
        return pca.channels[channel].duty_cycle

# Function responsible for mutating an individual channel for the PWM
def changeDuty(channel, dc):
    if (channel >= 0 and channel <= 15) and (dc >= 0 and dc <= 0xffff):
        pca.channels[channel].duty_cycle = dc

# Function responsible for returning the current value of the frequency of the PCA
def currentFrequency():
    return pca.frequency

# Function responsible for mutating the frequency (Hz) of the PCA9685
# May affect ability for it to communicate with some modules
# Valid integers of 40 and 1600 inclusive
def changeFrequency(frequency):
    if frequency >= 40 and frequency <= 1600:
        pca.frequency = frequency

# Output class for exclusively the LED
# Inherits the outputPort
class led(outputPort):

    # Constructor for the led
    def __init__(self, portNum):
        super().__init__(portNum)

    # Function responsible for slowly brightening and dimming the LED
    # Takes some time to complete as it goes through 16-bits
    def onAndOff(self):
        for i in range(0xffff):
            self.pwm = i
            self.channel.duty_cycle = i

        for i in range(0xffff, 0, -1):
            self.pwm = i
            self.channel.duty_cycle = i

    # Function associated with turning on or off the LED based on keywords 'on' and 'off'
    def ledStatus(self, instruction):
        if instruction.lower() == 'on':
            self.pwm = 0xffff
            self.channel.duty_cycle = 0xfff
        elif instruction.lower() == 'off':
            self.pwm = 0
            self.channel.duty_cycle = 0
        else:
            raise ValueError("Not a valid instruction!")

    # Function associated with turning on the LED to a specific brightness, using the duty cycle to
    # gauge the amount of voltage that would be received through the PWM
    # Accepted values only from 0 to 255, which is then mapped to 0 to ~1023
    def ledLevel(self, level):
        if (level >= 0 and level <= 256):
            self.pwm = level * 64
            self.channel.duty_cycle = level * 64
        else:
            raise ValueError("Not a valid PWM level")

    # Function to simply blink the LED an inputted number of times for a specified duration to be on during the blink
    # Number of blinks cannot be less than 0 and the duration cannot be in negative time
    def blink(self, blinks, duration):
        if (blinks >= 0 and duration > 0):
            for i in range(blinks):
                self.pwm = 0
                self.channel.duty_cycle = 0
                delay(10)

                self.pwm = 0xffff
                self.channel.duty_cycle = 0xffff

                delay(duration)

                self.pwm = 0
                self.channel.duty_cycle = 0
        else:
            raise ValueError("Blinks and duration must be greater than 0!")

# Output class for exclusively the servo device
# Inherits the outputPort
class servo(outputPort):

    # Constructor for the servo
    def __init__(self, portNum):
        super().__init__(portNum)

    # Function responsible for rotating the servo to a specific angle (a range of 0 to 180 degrees)
    def rotateTo(self, angle):
        # The approximations at the moment are difficult to control smoothly, will investigate better mapping
        # 0 degrees  approximately 2200 in duty-cycle
        # 180 degrees approximately 10300 in duty-cycle
        if angle >= 0 and angle <= 180:
            # The range is about 8,100, and the range for the angular movement is approximately 180
            # Hence, 8,100 / 180 = 45
            # Currently it is very hacky, and I'd like to make the movement more smooth
            self.pwm = math.ceil(angle * 45) + 2200
            self.channel.duty_cycle = self.pwm
        else:
            raise ValueError("Value of angle must be between 0 and 180 inclusively!")

# Output class for exclusively the buzzer device
# Inherits the outputPort
class buzzer(outputPort):

    def __init__(self, portNum):
        super().__init__(portNum)

    # Takes in a specific musical note (A - G) and 'buzzes' the corresponding note
    # WIP
    def playKey(self, note):
        # A, Ab, A#, B, Bb, B#, C, Cb, C#, D, Db, D#, E, Eb, E#, F, Fb, F#, G, Gb, G#
        return

#######
# MOTOR OUTPUT (PCA9685 & TB6612FNG)
#######

# Motor class is responsible for another section of the PWM output, but in this case, the signals
# for PWM 0 to 5. These signals will go to the TB6612 FNG Motor Driver and allow it to run based
# on the strength of the signal it receives (or the duration of the duty cycle since it is a PWM)
# PWM 0,1 are the Inputs for Motor 1, with PWM 2 being the 'speed' of Motor 1
# PWM 3,4 are the Inputs for Motor 2, with PWM 5 being the 'speed' of Motor 2

# The motor driver accepts the first two PWM signals as high or low inputs (PWM 0,1,3,4)
# These signals indicate which direction the motor will move (one of the two will be high and vice versa)

# The third PWM signal is responsible for the exact speed it receives based on the duty-cycle

# Motor class responsible for the motor drivers and its movement
class motor:

    # Variables associated with the motor pair number and the channels used for that motor pair
    motorNum = -1
    channel1 = -1
    channel2 = -1
    pwmChannel = -1

    # Constructor for the motor
    def __init__(self, motorNum):
        # Only accepted values for the motor number is 1 and 2 (yes, we should start at 0)
        if (motorNum > 0 and motorNum < 3):

            self.motorNum = motorNum

            # Brute force mapping of the motor pair and the respective channels
            # First pair uses PWM 0, 1, and 2
            # Second pair uses PWM 3, 4, and 5
            if (motorNum == 1):
                self.channel1 = pca.channels[0]
                self.channel2 = pca.channels[1]
                self.pwmChannel = pca.channels[2]
            elif (motorNum == 2):
                self.channel1 = pca.channels[3]
                self.channel2 = pca.channels[4]
                self.pwmChannel = pca.channels[5]
            #print('valid motor driver for ', motorNum)

        else:
            raise ValueError("Invalid motor number!")

    # Function for using the motor driver to move
    # The absolute value of the speed inputted will map to the correct analog value for the PWM to control the motor
    # If it is positive, it will rotate the motor clockwise
    # If it is negative, it will rotate the motor counter-clockwise
    # Accepted values only range from -256 to 256, exclusive
    # The duty-cycle accepts 0 to about 0x7fff, as the speed of the motor becomes almost dangerous as we approach
    # full 16-bit resolution of 0xffff
    def motorSpeed(self, speed):
        if (speed >= 0 and speed < 256):
            self.channel1.duty_cycle = 0
            self.channel2.duty_cycle = 0x7fff
            self.pwmChannel.duty_cycle = speed * 125
        elif (speed > -256 and speed < 0):
            self.channel1.duty_cycle = 0x7fff
            self.channel2.duty_cycle = 0
            self.pwmChannel.duty_cycle = speed * -125
        else:
            print('Invalid speed.')
            self.channel1.duty_cycle = 0
            self.channel2.duty_cycle = 0
            self.pwmChannel.duty_cycle = 0

    # Function for completely stopping the motor drivers, simply by setting all the channels to 0
    # Acts as a de facto 'clear' function as it halts all of the channels
    def motorStop(self):
        self.pwmChannel.duty_cycle = 0
        self.channel1.duty_cycle = 0
        self.channel2.duty_cycle = 0

# CONVENIENT MOTOR FUNCTIONS
# NEED TO BE TESTED


def motorsForward(motor1, motor2, speed):
    if (not(motor1 and motor2) is None) and (speed >= 0 and speed < 256):
        motor1.motorSpeed(speed)
        motor2.motorSpeed(speed)
    else:
        raise ValueError("Invalid speeds inputted!")

def motorsBackward(motor1, motor2, speed):
    if (not(motor1 and motor2) is None) and (speed >= 0 and speed < 256):
        motor1.motorSpeed(-speed)
        motor2.motorSpeed(-speed)
    else:
        raise ValueError("Invalid speeds inputted!")

def motorsLeft(motor1, motor2, speed1, speed2):
    if (not(motor1 and motor2) is None) and ((speed1 and speed2) >= 256 and (speed1 and speed2) < 256):
        motor1.motorSpeed(speed1)
        motor2.motorSpeed(-speed2)
    else:
        raise ValueError("Invalid speeds inputted!")

def motorsRight(motor1, motor2, speed1, speed2):
    if (not(motor1 and motor2) is None) and ((speed1 and speed2) >= 256 and (speed1 and speed2) < 256):
        motor1.motorSpeed(-speed1)
        motor2.motorSpeed(speed2)
    else:
        raise ValueError("Invalid speeds inputted!")

def motorsStop(motor1, motor2):
    if (not(motor1 and motor2) is None):
        motor1.motorStop()
        motor2.motorStop()
    else:
        raise ValueError("Invalid motors inputted!")

#######
# ULTRASONIC SENSOR INPUT/OUTPUT
#######

# STILL A WIP
# Using PWM for input for ultrasonic sensor is currently unviable and inconsistent - will move onto digital inputs
class ultrasonicSensor:

    # Validity flag to ensure proper port setup
    valid = True

    # Port assignment for the TRIGGER and ECHO (Output and Input respectively)
    # Channel will be associated with corresponding PWM channel
    trigPort = -1
    echoPort = -1
    channel = -1

    speedOfSound = 342.6    # metres/second

    # Constructor for the ultrasonic sensor class, accepting values for the input and output ports
    def __init__(self, trigPort, echoPort):

        # The object will only be valid if the ports selected are between 0 and 7 for both input and output
        if trigPort >= 0 and trigPort <= 7 and echoPort >= 0 and echoPort <= 7:
            self.valid = True

            self.echoPort = echoPort

            outputMapping = {0: 6,
                            1: 7,
                            2: 8,
                            3: 9,
                            4: 10,
                            5: 11,
                            6: 12,
                            7: 13}

            self.trigPort = outputMapping[trigPort]
            self.channel = pca.channels[self.trigPort]
        else:
            self.valid = False

    # Function to read in the ultrasonic sensor values ... but it doesn't work for some reason
    # What is the ideal duty cycle? What signal am I supposed to be receiving?
    def detect(self):
        self.channel.duty_cycle = 0
        delayMicro(2)
        self.channel.duty_cycle = 0xffff
        delayMicro(10)
        self.channel.duty_cycle = 0

        startTime = 0
        endTime = 0

        while not mcp.read_adc(self.echoPort) == 0xffff:
            startTime = time.time()

        while mcp.read_adc(self.echoPort) == 0xffff:
            endTime = time.time()

        duration = endTime - startTime

        distance = duration

        return distance

