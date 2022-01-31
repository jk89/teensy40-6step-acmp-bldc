# bldc-acmp
A bldc esc

# Circuit

The circuit is composed of a 3 phase inverter powered by 6 half bridges as well as IR2104 ic drivers which greatly simplifies the control design. BEMF measurements are taken by a voltage divider grid which allows for voltage measurements of the voltage on phase A, B, C phase channels, as well as a common virtual neutral. Zero crossing detection occurs when a phase signal crosses the virtual neutral signal.

You can see the circuit here: ./images/circuit.png or in the documentation ./notes/README.pdf

# Dependancies

1. jstest is what i've used to decode input from a PS4 controller. I tried pyPS4Controller but could not get it to work so built a parser for jstest output
	- sudo apt-get install joystick
2. pyserial is a library for sending command via usb to the teensy 4.0
	- (sudo [for global usage]) pip3 install -r pip.freeze 


# How to run

0. cd ./lib/control
1. Start sending commands from ps4 controller to remote: python3 remoteCommand.py
2. Starting the teensy command server python3 commandServer.py
3. Open SerialPlot select ttyACM0 and click open. FIXME
4. Press triangle on the PS4 controller to startup.
5. Press square on the PS4 controller to stop.
6. Squeeze right trigger to control thrust setting (WARNING as this is not working currently you could burn out hardware by using the throttle )

# Instructions without a PS4 controller.

You can use arduino ide to send commands to the controller directly. 

- Pressing triangle (start) corresponds to the command  b'\x14\x00\x00\x00\x00\x00\x00\x00\x00'
- Pressing x (start) corresponds to the command  b'\x00\x00\x00\x00\x00\x00\x00\x00\x00'

The first byte of a string of 9 for a full signal is the duty cycle setting. You can use this to increase the speed. Not you must start
with a thrust byte greater than that of the start command (20 in decimal, 14 in hex), lower than this and the ESC will use this as the signal to reset and shutdown the motor.

# Pin summary for teensy 40

The circuit has a female 13 pin header. Here is the configuration P1-x of the power circuit to teensy pin X:

	- P1-13 (A_IN): PIN 2
	- P1-12 (A_SD): PIN 1
	- P1-11 (B_IN): PIN 9
	- P1-10 (B_SD): PIN 0
	- P1-09 (C_IN): PIN 8
	- P1-08 (C_SD): PIN 7
	- P1-07: TEENSY/ARD GND
	- P1-06 (Phase A voltage divider signal): PIN 21
	- P1-05 (Phase B voltage divider signal): PIN 22
	- P1-04 (Phase C voltage divider signal): PIN 23
	- P1-03 (Virtual neutral voltage divider signal): PIN 18
	- P1-02 (Virtual neutral voltage divider signal): NOT USED
	- P1-01 (Virtual neutral voltage divider signal): NOT USED

# FlexPWM and ACMP information table:

| PHASE       | A   | B      | VN    | C     |
|-------------|-----|--------|-------|-------|
| PIN         | 21  | 22     | 18    | 23    |
| COLOUR      | RED | YELLOW | GREEN | BLACK |
| ACMP        | 1   | 2      | 1/2/3 | 3     |
| POLARITY    | +ve | +ve    | -ve   | +ve   |
| IN          | 6   | 5      | 5     | 0     |

ACMP has been set to trigger when each PWM channel counter reaches trigger point 4. This should theoretically syncronise ACMP samples such that they take place when the the transistors are not switching. This is to prevent reading the channel when it is set high/switching which would cause noisy BEMF measurements. 

# Motor information:

I am using the 1000KV Brushless Motor A2212 13T for testing. Other motors would work but you would have to tinker with the MIN_DUTY to ensure startup defeats the rolling resistance of the motor. Similarly you may have to experiment with the startup linear chirp to get it to start properly.

# WARNING
	This is an experimental project. I have destroyed a great deal of hardware (motors, transistors, ICs) and almost caused a fire once. Please be careful when using this code. ABSOLUTELY NO WARRANTY. If you do not agree to this then please do not download this software as this would violate the license which this code was distributed with, see ./LICENSE for details.

	Moreover this project lacks sensible precautions with the ESC to halt the motor when its in a faulty state (aka something is in the way of the motor spinning). I have cuts in my finger to prove it.

# Credits

This project was inspired greatly by the following arduino project: https://simple-circuit.com/arduino-sensorless-bldc-motor-controller-esc/
the circuit is largely the same but for some better available transistors and a modified BEMF voltage dividing grid to offer greater protection for teensy40(which is only 3v capable) when using larger motors.

https://www.youtube.com/watch?v=KxncxjNpMFI&

I have had great success with this project but wish to move to an ARM platform as it offers great features and power, hence the inception of this project.

# Improvements

Beyond the obvious of getting the current state working I have the following ideas for the future:

- The circuit is begging for some thermal and overdrive current protection 

- The ESC needs better fault protection logic, one should use the elapsed time between cycles to enforce some protection that if elapsed time deviates significantly from sensible values it could indicate that the motor has stalled or is obstructed, in this case current can surge and destroy hardware. Also this could maybe prevent it acting like a Dremel and risking your fingers with a higher kv motor.

# Copyright notice

Copyright Jonathan Kelsey 2021.
All code distributed with GNU GENERAL PUBLIC LICENSE Version 2.
The circuit diagram is freely available under MIT license ./images/circuit.png but this is the only file which is under MIT.
I will definitely consider changing the entire LICENSE to MIT when its working but this is at my discretion.