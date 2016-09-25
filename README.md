# Quadcopter-Arduino

## Goals
My goal is to build and program a quadcopter from scratch. It will feature:
  * a custom remote control
  * an Android App for controlling and displaying advanced options
  * an FPV system (First-person view, a live camera feed) (I will not code anything for this, but it will be included in the hardware overview)
  * and last but not least the quadcopter itself with an Adruino

## Project structure
I am currently working on a DIY quadcopter, which means building and programming it myself as far as possible. The hardware consists of three seperated systems:

### 1. The Quadcopter itself *(this repository)*:

The quadcopter consists of a main controller, the Arduino, which reads all sensor data, performs calculations on it and then controls the four motors. It also communicates with the remote control and controls secondary outputs, such as a camera gimbal or the lightning.

### 2. A Remote Control _(Coming soon)_:

_This part will be put into another repository, which I have not created yet._

### 3. An Android App _(Coming soon)_:

_This part will be put into another repository, which I have not created yet._

[Arduino]: https://github.com/JonasWanke/Quadcopter-Arduino
