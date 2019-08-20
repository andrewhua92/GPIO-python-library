
# sCHEESE Library

Python library for the CARobot's HAT for the Raspberry Pi dubbed the 'SwissCHEESE'

# Description

A work-in-progress. Developed for the purpose of simpler access and utilization of custom
modules from the SwissCHEESE HAT for the Raspberry Pi.

Uses the J8 Raspberry Pi GPIO Expansion, MCP3008 ADC, PCA9685 Driver, TB6612 FNG, and an EEPROM.
Utilizes several Adafruit Integrated Circuit libraries as well as other common Python libraries.

Intended usage is for simpler access to pin I/O from the HAT while using this module and as well 
as easier command of motor use, PWM functionality and additional purposes. 

The library has been developed thus far for the current prototype of the HAT, and 
will feature additional features as the HAT is updated.

# Features

The library is essentially a jack-of-all trades in terms of what it can provide to the Pi and
the HAT that is equipped.

## Input

Input is performed through the MCP3008, the Analog-to-Digital Converter (ADC), and communicates
with the Pi using SPI. The input class is able to create an object based on the selection of a 
valid input port, and then can read the current value it detects from the specified channel from
the chip. A total of 8 different input channels exist and can all be polled simulatenously.

As with the rest of the input classes, specific modules such as a button or potentiometer inherit the
input class and have their own set of specific and unique functions. 

## Output

Output is performed through the PCA9685, the PWM driver. The output class is able to
create an object based on the selection of a valid output port, and then can output
a specified duty-cycle, ranging from 0 to 0xffff (16-bit resolution). A total of 16 channels
can be sending out a signal, with 6 channels reserved for communication with the TB6612 FNG, 
and currently 2 unused. 

The output class has children of specific output modules, such as a buzzer or an LED, which 
have their own set of specific and unique functions.

## Motors

While a servo can be simply attached to an output port, motors need to be specifically attached
to the motor ports that communicate with the TB6612 FNG. They would also require a power source
that is powering the adapter on the HAT itself. The motor class itself requires a selection of
one of the two possible motor pairings, and allows for manipulation of speed, direction, and 
duration. It should be noted that motors do *not* inherit from output devices.