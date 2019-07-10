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

# Put import libraries on top

#########
# TIME FUNCTIONS
#########
import time

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

#######
# INPUT (MCP3008)
#######

# Input is directly involved from the input port on the hat and then goes through the Analog-to-Digital Converter,
# which is the MCP3008 IC. The input channels are respective to the input ports on the SwissCHEESE Hat, from A0 to
# A7. The output is read through SPI or Serial Peripheral Interface. A lot of the code was inherited from Adafruit's
# open source repository for the MCP3008 module.

import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008 as MCP

# Hardware SPI configuration
SPI_PORT = 0
SPI_DEVICE = 0
mcp = MCP.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

# Considering having individual input classes for different types of modules
# Input class for all general peripheral input devices - limited functions
class inputPort:

    # Validity flag to ensure that correct port is used
    # Considering instantly destructing this object is valid flag is not passed
    valid = True

    # Initialization of the port number being used
    portNum = -1

    # Constructor for the inputPort
    def __init__(self, portNum):
            # The only valid ports accepted will be from 0 to 7 (referred to as J1 to J8)
            if (portNum > -1 and portNum < 8) :
                self.portNum = portNum
                self.valid = True
            else:
                self.valid = False

    # Function to print the active value that is being read by the output IN ANALOG terms
    def printValue(self):
        if self.valid:
            print('Currently reading in a value of: ', mcp.read_adc(self.portNum))

    # Function responsible for simply returning the current value of the input (useful for conditionals)
    def currValue(self):
        if self.valid:
            return mcp.read_adc(self.portNum)

# Input class for exclusively the button
class button(inputPort):

    # Constructor for the inputPort
    def __init__(self, portNum):
        inputPort.__init__(self, portNum)

    # Function responsible for detecting whether a button input has been received
    # Can be used as a conditional statement to gate whether a button input has been received in the code
    def buttonPress(self):
        if self.valid:
            # Unsure about the correct analog value threshold to use as typically input goes above 1000
            if (mcp.read_adc(self.portNum) > 800):
                print('Button has been pressed')
                return True
            else:
                print('Button has not been pressed')
                return False

    # Function responsible for halting the code until the specified button has been pressed
    def untilButtonPress(self):
        if self.valid:
            while mcp.read.adc(self.portNum) < 800:
                # Does nothing until the button has been pressed (may incorporate a delay)
                self.valid = self.valid

class potentiometer(inputPort):

    def __init__(self, portNum):
        inputPort.__init__(self, portNum)

    def map(self, key):
        if self.valid:
            return math.ceil(mcp.read_adc(self.portNum)/1024 * key)

#######
# OUTPUT (PCA9685)
#######

# Output is directly involved with the output ports on the SwissCHEESE Hat. The output uses the SCL and SDA ports on
# the Pi (Serial Clock and Serial Data respectively) to send specific signals through one of the 16 channels on the
# PCA9685, which then sends the PWM (Pulse Width Modulation) signals to the respective output ports, whether it be
# actual output ports (J9 to J16) or the motor ports. The interface used is I2C or Inter-Integrated Circuit)
# Allegedly, the circuit has a 12-bit resolution, but is able to produce up to 16-bit resolution
# As a result, when setting PWM, your mileage may vary on the degree of success with the devices

from board import SCL, SDA
from adafruit_pca9685 import PCA9685
import busio

# Hardware specifications for the I2C
i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)

# Frequency associated with the PWM
# Too high of a frequency causes issues, so we settled for 60, which allows functionality
# for both LEDs and the motor drivers.
# Currently trying to determine the most optimal frequency for performance
pca.frequency = 60

# Output class for almost all general peripheral output devices - limited functions
class outputPort:

    # Validity flag to ensure the correct port is used
    valid = True

    # Constructor for the outputPort
    def __init__(self, portNum):
        # The only valid ports accepted will be from 0 to 7 (referred to as J9 to J16)
        if (portNum > -1 and portNum < 8):
            self.valid = True

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

            #print("PWM:", self.portNum)
        else:
            self.valid = False

    # Function responsible for a brute-force setting of the PWM to the device
    # Only accepts positive values of PWM by the PCA9685 (0 to 0xffff)
    def setPWM(self, pwm):
        if self.valid:
            if pwm >= 0:
                self.channel.duty_cycle = pwm
            else:
                print ('Invalid PWM.')

    # Function intended to clear the duty-cycle being sent through
    def clear(self):
        if self.valid:
            self.channel.duty_cycle = 0

# Output class for exclusively the LED
class led(outputPort):

    # Constructor for the led
    def __init__(self, portNum):
        outputPort.__init__(self, portNum)

    # Function responsible for slowly brightening and dimming the LED
    # Takes some time to complete as it goes through 16-bits
    def onAndOff(self):
        if self.valid:
            for i in range(0xffff):
                self.channel.duty_cycle = i

            for i in range(0xffff, 0, -1):
                self.channel.duty_cycle = i

    # Function associated with turning on or off the LED based on keywords 'on' and 'off'
    def ledStatus(self, instruction):
        if self.valid:
            if instruction.lower() == 'on':
                self.channel.duty_cycle = 0xfff
            elif instruction.lower() == 'off':
                self.channel.duty_cycle = 0
            else:
                print('Invalid instruction.')

    # Function associated with turning on the LED to a specific brightness, using the duty cycle to
    # gauge the amount of voltage that would be received through the PWM
    # Accepted values only from 0 to 255, which is then mapped to 0 to ~1023
    def ledLevel(self, level):
        if self.valid:
            if (level >= 0 and level <= 256):
                self.channel.duty_cycle = level * 16
            else:
                print('Invalid level.')

# Output class for exclusively the servo device
class servo(outputPort):

    # Constructor for the servo
    def __init__(self, portNum):
        outputPort.__init__(self, portNum)

    def rotateTo(self, angle):
        # 0 approx 2250 in duty-cycle
        # 180 approx 10250 in duty-cycle
        if self.valid:
            self.channel.duty_cycle = math.ceil(angle * 55.5) + 2250

#######
# MOTOR OUTPUT (TB6612FNG)
#######

# Motor class is responsible for another section of the PWM output, but in this case, the signals
# for PWM 0 to 5. These signals will go to the TB6612 FNG Motor Driver and allow it to run based
# on the strength of the signal it receives (or the duration of the duty cycle since it is a PWM)
# PWM 0,1 are the Inputs for Motor 1, with PWM 2 being the 'speed' of Motor 1
# PWM 3,4 are the Inputs for Motor 2, with PWM 5 being the 'speed' of Motor 2

# Motor class responsible for the motor drivers and its movement
class motor:

    # Variables associated with the motor pair number and the channels used for that motor pair
    motorNum = -1
    channel1 = -1
    channel2 = -1
    pwmChannel = -1

    # Validity flag to ensure the correct port is used
    valid = True

    # Constructor for the motor
    def __init__(self, motorNum):
        # Only accepted values for the motor number is 1 and 2 (yes, we should start at 0)
        if (motorNum > 0 and motorNum < 3):
            self.valid = True

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
            self.valid = False

    # Function for using the motor driver to move
    # The absolute value of the speed inputted will be mapped to the correct analog value for the PWM to control the motor
    # If it is positive, it will rotate the motor clockwise
    # If it is negative, it will rotate the motor counter-clockwise
    # Accepted values only range from -256 to 256, exclusive
    def motorSpeed(self, speed):
        if self.valid:
            if (speed >= 0 and speed < 256):
                self.channel1.duty_cycle = 0
                self.channel2.duty_cycle = 0x7fff
                self.pwmChannel.duty_cycle = speed * 125
            elif (speed > -256 and speed < 0):
                self.channel1.duty_cycle = 0x7fff
                self.channel2.duty_cycle = 0
                self.pwmChannel.duty_cycle = speed* -125
            else:
                print('Invalid speed.')
                self.channel1.duty_cycle = 0
                self.channel2.duty_cycle = 0
                self.pwmChannel = 0

    # Function for completely stopping the motor drivers, simply by setting all the channels to 0
    # Acts as a de facto 'clear' function as it halts all of the channels
    def motorStop(self):
        if self.valid:
            self.pwmChannel = 0
            self.channel1.duty_cycle = 0
            self.channel2.duty_cycle = 0

#######
# ULTRASONIC SENSOR INPUT/OUTPUT
#######

# Math module imported to use ceiling / floor functions for proper quantification of analog values from the sensor
import math

# STILL A WIP
class ultrasonicSensor:

    # Hardware specifications for the I2C
    i2c_bus = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c_bus)

    # Frequency associated with the PWM
    # Too high of a frequency causes issues, so we settled for 60, which allows functionality
    # for both LEDs and the motor drivers.
    # Currently trying to determine the most optimal frequency for performance
    pca.frequency = 60

    # Validity flag to ensure proper port setup
    valid = True

    # Port assignment for the TRIGGER and ECHO (Output and Input respectively)
    # Channel will be assoicated with corresponding PWM channel
    trigPort = -1
    echoPort = -1
    channel = -1

    # Constructor for the ultrasonic sensor class, accepting values for the input and output ports
    def __init__(self, trigPort, echoPort):

        # The object will only be valid if the ports selected are between 0 and 7 for both input and output
        if (trigPort >= 0 and trigPort <= 7 and echoPort >= 0 and echoPort <= 7):
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
            self.channel = self.pca.channels[self.trigPort]
        else:
            self.valid = False

    def detect(self):
        self.channel.duty_cycle = 0
        delayMicro(2)
        self.channel.duty_cycle = 0xfff
        delayMicro(10)
        self.channel.duty_cycle = 0

        startTime = 0
        endTime = 0

        while mcp.read_adc(self.echoPort) == 0xfff:
            startTime = time.time()

        while not mcp.read_adc(self.echoPort) == 0xfff:
            endTime = time.time()

        duration = endtime - startTime

        distance = duration

        return distance
