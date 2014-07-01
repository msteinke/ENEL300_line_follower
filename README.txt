This project is to build a line following robot for the ENEL300 course. Code from Michael Hayes' UCFK4 is may be used in this project.


==========================================================
How to use the code on Windows 7
==========================================================
Install AtmelStudio 6.2 (and Flip 3.4.7). Also install
GitHub for Windows from windows.github.com.

----------------------------------------------------------
To Setup Git:
----------------------------------------------------------





==========================================================
UCFK4 Documentation
==========================================================

For documentation, 
see http://ecewiki.elec.canterbury.ac.nz/mediawiki/index.php/UCFK4

The directory structure is:
apps         --- contains a sub-directory for each application
drivers      --- device driver modules (hardware independent)
drivers/avr  --- device driver modules specifically for AVR architecture
drivers/test --- device driver modules for test scaffold
doc          --- documentation
etc          --- miscellaneous scripts and makefile templates
fonts        --- fonts and font creation program
utils        --- utility modules

UCFK4 expansion connector (numbered from the end closest
to the USB connector):
1  GND
2  VDD (5 V if USB powered)
3  PD1
4  PD2/RXD1 (this is shared with IR receiver)
5  PD3/TXD1 (this is shared with IR LED)
6  PD4
7  PD5
8  PD6

to reset UCFK4:
Push and hold S3 (HW3, PD7)
Push and release S2 (RESET, PC1)
Release S3 (HW3, PD7)





