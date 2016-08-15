# MotorLib_Embedded
Suite of library based code to provide Motor Control and MotionControl for Embedded Systems/MCUs. 

Provides libraries for Brushed DC (BDC), Stepper, and Brushless DC (BLDC/PMSM) motors, using a common higher level API. 

Includes support for Hall sensored and sensorless (BEMF) algorithms, PID for Speed Control and Torque Control,
Supports standard rotary encoders, and associated position control and motion control algorithms.

Supported MCUs include ST STM32, TI Tiva 123G and 129, MSP432, MSP430 (FR5969 initially), and Cypress PSoC4,
with examples for each.

Supports commonly available Motor Control Chips/Drivers including:
     BDC:  TN754410, L293D, Xnucleo L6206, and DRV8848 Boosterpack
     BLDC: Xnucleo L6230, L6398, and DRV8301, DRV8305 Boosterpacks, and PSoC4 CY8CKIT-037
     Stepper: Xnucleo L6474, L6470, PowerStep01, and DRV8711 Boosterpack

The library is portable across all of the above processors, so a Motor Control application written on one 
processor can be quickly ported to a different processor, using the same high level API.
Portability between processors is achieved via a set of low-level plug-in (BDC, BLDC, Stepper) drivers
for each supported platform.

Code is extremely well documented, so reading through the code is a good way to learn
about the inner details of motor control and motion control. Library is part of a
book on Embedded Motion Control that will be published in 2017.
