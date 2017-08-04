Group 51:
Frederick Fraser
Luke Derry

Final Version

TODO: [ UPDATED 29 MAY ]
- Separate out helicopter input.                                                    [Complete]  
- Look into buttons. Adopt and adapt buttons2.c to incoporate all input             [Complete]  
- Make PID control function return duty cycle (Percent 0.05 - 0.95)                 [Complete]  
- Make PID controller for tail and main, separate values for each may be req.       [Complete]                      
- Buttons change target yaw and target alt variables.                               [Complete]  
- Switch MODE to allow landing/flying/userInput sequences.                          [Complete]  
- UART and OLED. Method to sort output then send to both uart+oled separately       [Complete]  
- Make landing + startup sequence.                                                  [Complete]  
- Find forward reference                                                            [Complete]  
- Implement soft reset                                                              [Complete]  
- Make OLED/UART Display format adheres to specification                            [Complete]  
- Destroy freezing bug                                                              [Complete]
- UART/OLED Yaw display increment = [float 15.27] rather than specificed [int 15]   [Complete]
- Comments + WhiteSpace + Formatting                                                [Complete]







-
-
-
[History]
__ 28.3.2017 ___________________________________________________________________  
Updated Master branch to 28th March [Milestone1] version.  
This has YET to be tested in the lab.  
Updated / Uploaded:  
- Main.c  
- pulseInrpt.c + pulseInrpt.h  
- pwmGen.c + pwmGen.h  
- README.md  
  
NB: I assumed that the buttons2 and cirBufT files were not altered.  
________________________________________________________________________________  
  
.  
  
.  
  
.  
  
__ 24.3.2017 ___________________________________________________________________  
I made a new Branch.   
"NotVeryGood" branch is... kind of working... I put all the things into a single  
main file and got it to more or less work, but it was quite poorly done and I  
might try and redo most of it. It main problem (other than being messy) is that   
for some reason the intervals dont seem to be being recorded quite right and  
converting from intervals to frequency seems to only work when the input  
frequency is a multipul of 100... for some reason. But, all those problems  
aside, it is doing generally what its meant to.  
________________________________________________________________________________  
  
.  
  
.  
  
.  
  
__ 21.3.2017 ___________________________________________________________________  
TODO: (loose guide - feel free to change any/all!)  
  
1. Create main .c file and write empty (required) methods.  
    - Such as init() main() display() etc.. [Completed]  
      
2. Strip out functionality of pwmGen.c and pulseInrpt.c and _somehow_  
    make this callable from our main .c file. [Completed]  
________________________________________________________________________________  
  
.  
  
.  
  
.  
  
________________________________________________________________________________  

MILESTONE 1 REQUIREMENTS:  
________________________________________________________________________________  
  
M1.1 Generate a PWM output on Connector J4-05 (PC5, M0PWM7).  
  
M1.2 Initialise the frequency of the PWM output to 150 Hz and the duty cycle of   
the PWM output to 50%.  
  
M1.3 Control the duty cycle of the PWM output (monitor this by oscilloscope) by  
means of the UP and DOWN pushbuttons on the Orbit daughterboard: each push of   
the UP button increases the duty cycle by 5%, up to a maximum of 95%; each push  
of the DOWN button decreases the duty cycle by 5%, down to a minimum of 5%;  
additional pushes which seek to go outside the range 5 to 95% are ignored.  
  
M1.4 The button pushes should be properly debounced – one push, one action.  
  
M1.5 Tiva Pin J1-03 (PB0) should be programmed for pin-change interrupts; by  
measuring the period of the 0 – 3 V pulsed (i.e. 2-level) waveform input (if  
such a waveform is present) on Pin J1-03, the PWM output frequency should change  
to match the input waveform frequency. Thus it is effectively “entrained” in  
frequency, although not synchronous. The duty cycle of the PWM should remain  
under button control.  
  
M1.6 If a suitable waveform (i.e. as specified in M1.5) is not present on Pin  
J1-03, the PWM output should revert to 150 Hz.  
  
M1.7 The operating frequency range for the entrainment is 100 to 300 Hz.  
  
M1.8 The current frequency and duty cycle of the PWM output should be displayed  
reliably and stably on the Orbit OLED display.  
  
  
