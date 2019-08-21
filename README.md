
# sCHEESE Library

Python library for the CARobot's HAT for the Raspberry Pi dubbed the 'SwissCHEESE'

# Description

A work-in-progress. Developed for the purpose of simpler access and utilization of custom
modules from the SwissCHEESE HAT for the Raspberry Pi.

Uses the J8 Raspberry Pi GPIO Expansion, MCP3008 ADC, PCA9685 Driver, TB6612 FNG, and an EEPROM.
Utilizes several Adafruit Integrated Circuit libraries as well as other common Python libraries.

Intended usage is for simpler access to pin I/O from the HAT while using this module and as well 
as easier command of motor use, PWM functionality and additional purposes. A lot of inspiration 
is drawn from the available Arduino libraries in how they handle I/0 functionality, as well
as the _gpiozero_ library in ideas for what specific components should do.

The library has been developed thus far for the current prototype of the HAT, and 
will feature additional features as the HAT is updated.

# Features

The library is essentially a jack-of-all trades in terms of what it can provide to the Pi and
the HAT that is equipped. It also comes with delay functions similar in functionality that
Arduino has (delay in milliseconds, delayMicro in microseconds)

> import sCHEESE as sc
>
> \# set the duty cycle on port 1 to be half duty-cycle
>
> opt = sc.output(1)
>
> opt.setPWM(0x7fff)
>
> \# prints the vendor, product, and ID info of the HAT
>
> sc.printInfo()
>
> \# make an LED object, and turn the corresponding LED on and off
>
> led = sc.led(5)
> 
> led.onAndOff()

## Input

Input is performed through the MCP3008, the Analog-to-Digital Converter (ADC), and communicates
with the Pi using SPI. The input class is able to create an object based on the selection of a 
valid input port, and then can read the current value it detects from the specified channel from
the chip. A total of 8 different input channels exist and can all be polled simulatenously. Note
that the channels are numbered 0 to 7. The input ports are also numbered 0 to 7.

> import sCHEESE as sc
>
> btn = sc.button(2)
>
> \# function to wait for button input before proceeding with rest of script
>
> btn.untilButtonPress()
>
> ...


As with the rest of the input classes, specific modules such as a button or potentiometer inherit the
input class and have their own set of specific and unique functions. 

## Output

Output is performed through the PCA9685, the PWM driver. The output class is able to
create an object based on the selection of a valid output port, and then can output
a specified duty-cycle, ranging from 0 to 0xffff (16-bit resolution). A total of 16 channels
can be sending out a signal, with 6 channels reserved for communication with the TB6612 FNG, 
and currently 2 unused. Note that the channels are numbered 0 to 15. However, the output
ports themselves are numbered 0 to 7, and are mapped accordingly to the PCA9685. 

> import sCHEESE as sc
>
> \# initialize an LED object on output port 2
>
> led = sc.led(2)
>
> \# turn on the LED to a brightness of 200 out of 255
>
> led.ledLevel(200)

The output class has children of specific output modules, such as a buzzer or an LED, which 
have their own set of specific and unique functions.

## Motors

While a servo can be simply attached to an output port, motors need to be specifically attached
to the motor ports that communicate with the TB6612 FNG. They would also require a power source
that is powering the adapter on the HAT itself. The motor class itself requires a selection of
one of the two possible motor pairings, and allows for manipulation of speed, direction, and 
duration. It should be noted that motors do *not* inherit from output devices.