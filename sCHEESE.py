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

class inputPin: # inputPin will allow user to assign a pin for input purposes(reads in something from the outside)
    
    valid = True
    
    def __init__(self, pinNum):  
        #import RPi.GPIO as GPIO  consider moving import RPi.GPIO out of the init for the classes 
        self.valid = True
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BOARD)
        self.GPIO = GPIO
        
        # maps the pins for the HAT's connections to the J8 RPi numbering system
        mapping = { 0 : 29,
                                1 :  31,
                                2 :  33,
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
    
    def pinRead(self):
        if self.valid:
            print('you reading')
            return self.GPIO.input(self.pinNum); # not sure if  I should use self. prefix or not
        else:
            print('not the right word')    
    
    def clean(self):
        self.GPIO.cleanup()

# it's possible the entire output / input pin can be integrated into one class, will depend on the HAT's capabilities
class outputPin:

    valid = True

    def __init__(self, pinNum):
        self.valid = True
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BOARD)
        self.GPIO = GPIO
        
        mapping = { 0 : 29,
                                1 :  31,
                                2 :  33,
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
                self.GPIO.output(self.pinNum, self.GPIO.HIGH)
                print('this is high')
            elif state.lower() == 'low':
                self.GPIO.output(self.pinNum, self.GPIO.LOW)
                print('this is low')
            else:
                print('not the right word')
        
    def clean(self):
        self.GPIO.cleanup()            

#########
# PWM WITH I2C
#########


from board import SCL, SDA
import busio

from adafruit_pca9685 import PCA9685

# generates the I2C bus interface
i2c_bus = busio.I2C(SCL,SDA)

# pca object for the circuit
pca = PCA9685(i2c_bus)

pca.frequency = 60

#########
# SERVO
#########

from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)