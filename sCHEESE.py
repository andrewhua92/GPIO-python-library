# TODO:
# work on PWM implementation
# preventative measures for single mapping per pin
# work on motor control
# figure out how the MOSI & MISO work ...

#########
# TIME
#########
import time

def delay(milliseconds):
    time.sleep(milliseconds / 1000)
    print('time delay of  {0} '.format(milliseconds/1000))

#########
# BASIC INPUT/OUTPUT (IO)
#########

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

# outputPin will allow user to assign a pin for outputpurposes(reads in something from the outside)
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
# PWM WITH I2C
#########


### test when we have the board ready
'''
import board
import busio

import adafruit_pca9685

# generates the I2C bus interface
i2c_bus = busio.I2C(board.SCL,board.SDA)

# pca object for the circuit
pca = adafruit_pca9685.PCA9685(i2c_bus)

pca.frequency = 60

#########
# SERVO
#########

import adafruit_servokit

kit = ServoKit(channels=16)
'''

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
        
#########
# ADC (Analog-to-Digital-Converter)
#########


