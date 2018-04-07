# R2 Robotics
Repo for the 2R Educational Robot of ME 495 Robot Design Studio

# System capabilities
* Timers: 6 16/32-bit blocks, each block has 2 16-bit counters (timer A and timer B) that can be combined to 32-bit, and 6 wide 32/64-bit blocks, each wide block has 2 32-bit counters that can be combined to 64 bits. Note that some timers are used for PWM.

# Examples
The examples include short code snippets to quickly get use out of the TM4C123GXL Launchpad
* Blink
* PWM
* PID Controller (QEI + PWM)
* Quadrature Encoder Interface (QEI)
* Debug Output via UART
* SPI
* SPI RAM (todo)
* UART


# Roadmap
This roadmap lists the tasks to do as the software architecture develops
* PID control for 1 motor
* PID control for 2 motors
* MATLAB interface for PID for 1 motor
* MATLAB interface for PID for 2 motor
* Lead-Lag control law
* GUI interface for PID control
* Velocity PID control
* Implement control law interface
* Stopping code - motor spins very fast when loading new code at the moment
* ~~Implement modifiable control law interface~~
* ~~Implement sensor SPI interface~~
* ~~Current sense code~~
* ~~Temperature sense code~~



# Quickstart

We found that when starting a project, these libraries have to be manually added. If there is an automatic way, feel free to share it with us!

*[Left click on Project] -> Show Build Settings -> CSS Build, ARM Compiler, Include options -> add "C:\ti\TivaWare_C_Series-2.1.3.156" to path*

*[Left click on Project] -> Show Build Settings -> CSS Build, ARM Linker, Include lib -> add "C:\ti\TivaWare_C_Series-2.1.3.156\driverlib\ccs\Debug\driverlib.lib"*

*[Right click on Project] -> Import, Import -> General, Import from File System -> "C:\ti\ivaWare_C_Series-2.1.3.156\utils", Select the ones you want*

## Making the stack size bigger
If you want to use string processing/more breakpoints

*[Right click on Project] -> Arm Linker, Basic Options -> set stack size to 1024*


## Including the R2R header and source files
*[Right click on Project] -> New, Header file -> name file "r2r.h" -> copy and paste .r and .c code from repo*

## Adding interrupts
1. Enable interrupts
2. Write interrupt code
3. Define interrupt in the startup.ccs file
4. Define interrupt in the interrupt vector

## Declarations
Using '=' outside of the main loop will generate an error because CCS will think it is a declaration and be confused if you previously declared the variable.

# Useful References
1. [Tivaware Driver Peripheral Library Reference](http://www.ti.com/lit/ug/spmu298d/spmu298d.pdf)