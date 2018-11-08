## Table of Contents 

- [DC Motor Control](#dc-motor-control)

## DC Motor Control 

For this project the car has three motors. One of them is to control the direction using a rack and pinion mechanism. The other two motors give the force for the motion of the car, creating an electric differential. This model of the car has rear wheel drive. 

In order to have the position of the front-wheels with an error less than 0.1%, a PID controller was created, using a quadrature encoder before the gear box of the DC motor and the PIC18F4431 Microchip Microcontroller wich has a special module for motion measurement called QEI.


