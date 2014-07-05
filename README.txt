This project is to build a line following robot for the 
ENEL300 course. Code from Michael Hayes' UCFK4 is may be 
used in this project.


==========================================================
 How to use the code on Windows 7
==========================================================
	Install AtmelStudio 6.2 (and Flip 3.4.7). Also install
	Git. I used GitHub for windows from : windows.github.com.
	I haven't figured out how to use the GUI yet, but a 
	shell does come with it.

	Once the code compiles and produces a .hex file in 'Debug'
	folder, use Flip to connect to the board then load the
	hex file.

	Here's what I do:
	Make the board ready in three steps:
	Push and hold S3 (HW3, PD7)
	Push and release S2 (RESET, PC1)
	Release S3 (HW3, PD7)

	Then I click on line_follower.bat. You
	can add this as an 'external tool' in 
	atmel studio so that you don't need to
	switch between windows.


==========================================================
 PIN-OUT
==========================================================
	Comparator:
	PIN | NAME | DESCRIPTION
	PD1 | AIN0 | Positive input to comparator.
	PD2 | AIN1 | Negative input to comparator (via mux).
	PC2 | AIN2 | Negative input to comparator (via mux).
	PD4 | AIN3 | Negative input to comparator (via mux).
	PD5 | AIN4 | Negative input to comparator (via mux).
	PD6 | AIN5 | Negative input to comparator (via mux).
	PD7 | AIN6 | Negative input to comparator (via mux).

	Pulse Width Modulator:
	PIN | NAME | DESCRIPTION
	PB7 | OC0A | Timer0 PWM channel A.
	PD0 | OC0B | Timer0 PWM channel B.
	PC6 | OC1A | Timer1 PWM channel A.
	PC5 | OC1B | Timer1 PWM channel B.
	PB7 | OC1C | Timer1 PWM channel C. * not implemented

	Other Releavent Pins:
	PIN | NAME | DESCRIPTION
	PD7 | HW3  | Hardware pin for bootloader
	PC1 | RESET| Reset pin

==========================================================
 Modules Written for Line-follower
==========================================================
	NAME 				| DESCRIPTION
	line_follower.c		| main project file
		motor			| module to control motors with PWM
		sensor			| module to read in line detection sensors
			comparator	| abstraction module for analog comparator
		system			| module to contain pin allocations, structs, types etc

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





