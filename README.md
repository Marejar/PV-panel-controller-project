# PV-panel-controller-project
This repository contains photovoltaic panel position controller.

# Description

This project was created to make a model od device, which will find optimal position for PV panel, to let it work in optimal work point. Main parts of this project are two servo motors which are changing position of PV panel. There are also four analog light sensors nearly PV panel. The best position of panel is when light intensity is the same on every sensor which is familiar to when the voltage of the sensor is the same. That's why four ADC channels were used to gather voltage from sensors. That data is processed by alghoritm and position of desired servo motor is changed. This happend until optimal position is set.

It is possible to set position of servo from mobile phone via bluetooth. For this HC-05 module was used. It is need to change program state to "REMOTE". After this it is possible to freely select position of servos. LCD display shows actual position of servo motors.

#PCB Layout for this project
![image](https://user-images.githubusercontent.com/83860327/147876328-8f0c8b10-81be-429b-9eb6-ad5a81b95e12.png)

## Parts

This project is customized to use with STM32F446RE module. 
Elements used in this project:
-STM32F446RE board
-Servo motors
-LCD 2x16
-analog light sensors
-HC-05 bluetooth module
-PV panel

This project uses HAL libraries, and Mohamed Yaqoob's LCD library 

## Authors

* **Marek Jaromin** - *Initial work* 
