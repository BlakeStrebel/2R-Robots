# 2R-Robots
Repo for the 2R Educational Robot of ME 495 Robot Design Studio


# Examples
The examples include short code snippets to quickly get use out of the TM4C123GXL Launchpad
* Blink
* PWM
* PID Controller (QEI + PWM)
* Quadrature Encoder Interface (QEI)
* Debug Output via UART
* SPI
* SD Card (todo)
* UART (todo)
* CAN (todo)


# Quickstart

We found that when starting a project, these libraries have to be manually added. If there is an automatic way, feel free to share it with us!

*[Left click on Project] -> Show Build Settings -> CSS Build, ARM Compiler, Include options -> add "C:\ti\TivaWare_C_Series-2.1.3.156" to path*

*[Left click on Project] -> Show Build Settings -> CSS Build, ARM Linker, Include lib -> add "C:\ti\TivaWare_C_Series-2.1.3.156\driverlib\ccs\Debug\driverlib.lib"*

*[Right click on Project] -> Import, Import -> General, Import from File System -> "C:\ti\ivaWare_C_Series-2.1.3.156\utils", Select the ones you want*