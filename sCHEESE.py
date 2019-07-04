# Python Library for the SwissCHEESE Hat made by CARobotics
# Developed by Andrew Hua
# Credits go to the respective owners of the imported libraries

#########
# TIME FUNCTIONS
#########
import time

# The delay function will mimic the delay function which exists in an Arduino, although it is entirely feasible
# to also simply use time.sleep() if the user wishes to associate delays in larger units of time (i.e. seconds)

def delay(milliseconds):
    time.sleep(milliseconds / 1000)
    print('time delay of  {0} millisecond(s)'.format(milliseconds/1000))

# The delayMicro function will mimic the same function in Arduino, with the purpose of being even shorter
# amount of time, intended for unique circumstances and hardware

def delayMicro(microseconds):
    time.sleep(microseconds/1000000)
    print('time delay of {0} microsecond(s)'.format(microseconds/1000000))

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

class inputPort:

    # Validity flag to ensure that correct port is used
    # Considering instantly destructing this object is valid flag is not passed
    valid = True

    # Initialization of the port number being used
    portNum = -1;

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
        if valid:
            print('Currently reading in a value of: ', mcp.read_adc(self.portNum))

    # Function responsible for detecting whether a button input has been received
    # Can be used as a conditional statement to gate whether a button input has been received in the code
    def buttonPress(self):
        if valid:
            # Unsure about the correct analog value threshold to use as typically input goes above 1000
            if (mcp.read_adc(self.portNum) > 800):
                print('Button has been pressed')
                return True
            else:
                print('Button has not been pressed')
                return False

    # Function responsible for simply returning the current value of the input (useful for conditionals)
    def currValue(self):
        if valid:
            return mcp.read_adc(self.portNum)

#######
# OUTPUT (PCA9685)
#######

# Output is directly involved with the output ports on the SwissCHEESE Hat. The output uses the SCL and SDA ports on
# the Pi (Serial Clock and Serial Data respectively) to send specific signals through one of the 16 channels on the
# PCA9685, which then sends the PWM (Pulse Width Modulation) signals to the respective output ports, whether it be
# actual output ports (J9 to J16) or the motor ports. The interface used is I2C or Intger-Integrated Circuit)

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

class outputPort:

    # Port Number (on the HAT for Output) and the PWM Channel associated with the port
    portNum = -1
    channel = -1

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

            print("PWM:", self.portNum)
        else:
            self.valid = False

    # Testing function to see if the LED attached can turn on and off
    def onAndOff(self):
        if valid:
            for i in range(10000):
                self.channel.duty_cycle = i

            for i in range(10000, 0, -1):
                self.channel.duty_cycle = i

    # Function associated with turning on or off the LED
    def ledStatus(self, instruction):
        if valid:
            if instruction.lower() == 'on':
                self.channel.duty_cycle = 0x7fff
            elif instruction.lower() == 'off':
                self.channel.duty_cycle = 0
            else:
                print('invalid instruction')

    # Function associated with turning on the LED to a specific brightness, using the duty cycle to
    # gauge the amount of voltage that would be received through the PWM
    # Accepted values only from 0 to 255, which is then mapped to 0 to ~1023
    def ledLevel(self, level):
        if valid:
            if (level >= 0 && level < 256)
                self.channel.duty_cycle = level * 256
            else:
                print('invalid level')

#######
# MOTOR OUTPUT (TB6612FNG)
#######

# Motor class is responsible for another section of the PWM output, but in this case, the signals
# for PWM 0 to 5. These signals will go to the TB6612 FNG Motor Driver and allow it to run based
# on the strength of the signal it receives (or the duration of the duty cycle since it is a PWM)
# PWM 0,1 are the Inputs for Motor 1, with PWM 2 being the 'speed' of Motor 1
# PWM 3,4 are the Inputs for Motor 2, with PWM 5 being the 'speed' of Motor 2

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

            # Brute force mapping of the motor pair and the respective channels
            # First pair uses PWM 0, 1, and 2
            # Second pair uses PWM 3, 4, and 5
            if (motorNum == 1):
                self.channel1 = pca.channels[0]
                self.channel2 = pca.channels[1]
                self.pwmChannel = pca.channels[2]
            else:
                self.channel1 = pca.channels[3]
                self.channel2 = pca.channels[4]
                self.pwmChannel = pca.channels[5]

        else:
            self.valid = False

    # Function for using the motor driver to move
    # The absolute value of the speed inputted will be mapped to the correct analog value for the PWM to control the motor
    # If it is positive, it will rotate the motor clockwise
    # If it is negative, it will rotate the motor counter-clockwise
    # Accepted values only range from -256 to 256, exclusive
    def motorSpeed(self, speed):
        if valid:
            if (speed >= 0 and speed < 256):
                self.channel1.duty_cycle = 0
                self.channel2.duty_cycle = 0xffff
                self.pwmChannel = speed*256
            elif (speed > -256 and speed < 0):
                self.channel1.duty_cycle = 0xffff
                self.channel2.duty_cycle = 0
                self.pwmChannel = speed*256
            else:
                print('invalid speed')

    # Function for completely stopping the motor drivers, simply by setting all the channels to 0
    def motorStop(self):
        if valid:
            self.channel1.duty_cycle = 0
            self.channel2.duty_cycle = 0
            self.pwmChannel = 0

'''
# Will incorporate GPIO back into the library if needed

#########
# BASIC INPUT/OUTPUT (IO)
#########

# I/O section using the RPi.GPIO library
import RPi.GPIO as GPIO

# disables warning, not sure if this is bad or not
GPIO.setwarnings(False)

# it's possible the entire output / input pin can be integrated into one class, will depend on the HAT's capabilities
 
# inputPin will allow user to assign a pin for input purposes(reads in something from the outside)
class inputPin:
    
    valid = True
    
    # constructor
    def __init__(self, pinNum):  
        self.valid = True
        GPIO.setmode(GPIO.BOARD)
        
        # maps the pins for the HAT's connections to the J8 RPi numbering system
        mapping = { 0 : 29,
                    1 : 31,
                    2 : 33,
                    3 : 35,
                    4 : 37,
                    5 : 32,
                    6 : 36,
                    7 : 38}
        if pinNum >= 0 and pinNum <= 7:
            self.pinNum = mapping[pinNum]
            GPIO.setup(self.pinNum, GPIO.OUT);
        else:
            self.valid = False
    
    # reads in current electrical signal value from the assigned pin
    def pinRead(self):
        if self.valid:
            print('you reading')
            return GPIO.input(self.pinNum); # not sure if  I should use self. prefix or not
        else:
            print('not the right word')    
    
    # destructor
    def __del__(self):
        GPIO.cleanup()

# outputPin will allow user to assign a pin for output purposes(reads in something from the outside)
class outputPin:

    valid = True

    def __init__(self, pinNum):
        self.valid = True
        GPIO.setmode(GPIO.BOARD)
        
        mapping = { 0 : 29,
                    1 : 31,
                    2 : 33,
                    3 : 35,
                    4 : 37,
                    5 : 32,
                    6 : 36,
                    7 : 38}
        
        if pinNum >= 0 and pinNum <= 7:
            self.pinNum = mapping[pinNum]
            GPIO.setup(self.pinNum, GPIO.OUT);
        else:
            self.valid = False
    
    def pinWrite(self, state):
        if self.valid:
            if state.lower() == 'high':
                GPIO.output(self.pinNum, GPIO.HIGH)
                print('this is high')
            elif state.lower() == 'low':
                GPIO.output(self.pinNum, GPIO.LOW)
                print('this is low')
            else:
                print('not the right word')
        
    def __del__(self):
        GPIO.cleanup()

#########
# BASIC INPUT/OUTPUT (IO) 2.0
#########

# commanding I/O using the gpiozero library
# at the moment, not sure which library is better / compatible with the HAT
import gpiozero as gp0

class pinInit:

    valid = True

    def __init__(self, pinNum):
        self.pinNum = pinNum

    def led(self, status):
        diode = gp0.LED(self.pinNum)
        if state.lower() == 'high':
            diode.on()
        elif state.lower() == 'low':
            diode.off()
        elif state.lower() == 'blink':
            diode.blink()

    def button(self, pinNum):
        btn = gp0.BUTTON(self.pinNum)
        if btn.is_pressed:
            return True
        else:
            return False

    def waitForButton(self, pinNum):
        btn = gp0.BUTTON(self.pinNum)
        while btn.wait_for_press():
            time.sleep(1)

#########
# PWM WITH I2C
#########


### test when we have the board ready

import board
import busio

import adafruit_pca9685

# generates the I2C bus interface
i2c_bus = busio.I2C(board.SCL,board.SDA)

# pca object for the circuit
pca = adafruit_pca9685.PCA9685(i2c_bus)

pca.frequency = 60

#########
# MOTOR
#########

# Credit to nick-hunter on github for example code for a Python library for the TB6612FNG Motor

class motor:
    
    # input channels for the motors
    in1 = ''
    in2 = ''
    
    # pulse width modulation for the 'analog' value
    pwm = ''
    standbyPin = ''
    
    # frequency used for the PWM
    hertz = 1000
    
    # constructor for the motor object
    def __init__ (self, in1, in2, pwm, standbyPin):
        GPIO.setmode(board)
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.standbyPin = standbyPin
        self.reverse = reverse
        
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.pwm, GPIO.OUT)
        GPIO.setup(self.standbyPin, GPIO.OUT)
        self.p = GPIO.PWM(pwm, self.hertz)
        self.p.start(0)
        
    # function responsible for the movement of the motor (with speed between -100 and 100)
    def move(self, speed):
        dutyCycle = speed
        
        # initiates reverse movement
        if (speed < 0):
            dutyCycle = dutyCycle * -1
            
        # outputs movement of the motor
        if (speed > 0):
            GPIO.output(self.in1, GPIO.HIGH)
            GPIO.output(self.in2, GPIO.LOW)
        else:
            GPIO.output(self.in1, GPIO.LOW)
            GPIO.output(self.in2, GPIO.HIGH)
            
        self.p.ChangeDutyCycle(dutyCycle)
        
    # stops the motor from moving
    def stop(self):
        self.p.ChangeDutyCycle(0)
        GPIO.output(self.in1,GPIO.HIGH)
        GPIO.output(self.in2,GPIO.HIGH)
        
    # standby ?? not exactly sure what the purpose of this is, will remain to be seen if we will keep
    def standby(self):
        self.p.ChangeDutyCycle(0)
        GPIO.output(self.standbyPin, value)
        
    # destructor
    def __del__(self):
        GPIO.cleanup()
'''
