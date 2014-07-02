@ECHO OFF
title ATmega32U2 Programmer Script
REM mode CON cols=100 lines=30
REM device is: ATmega32U2
REM cd C:\

REM set line_follower=C:\Users\martin\Documents\Atmel Studio\6.2\line_follower\line_follower\Debug\line_follower
REM batchisp -device ATmega32U2 -hardware usb -operation erase f memory flash blankcheck loadbuffer line_follower.hex program verify start reset 0
batchisp -device ATmega32U2 -hardware usb -operation erase f memory flash blankcheck loadbuffer Debug\line_follower.hex program verify start reset 0

REM batchisp -device at32uc3a1128 -hardware usb -operation erase f memory flash blankcheck loadbuffer line_follower.hex program verify start reset 0

pause