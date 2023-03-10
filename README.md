# speed-imu-pub
ros publisher node of wheels speed and imu 

## Boards used
***STM32 NUCLEO-64 F410RE***    
IMU ***IKS01A2*** / ***IKS02A1***

In order to test the code connect the PINS PB13, PB14, PB1 and PB2 as it is shown in the figure, or directly to some encoders.

![image](https://user-images.githubusercontent.com/115342258/224169768-62fede09-32b7-47fc-b086-630190e35790.png)

Then open 3 terminals

In the first use the command ```roscore```

In the second the command ```rosrun rosserial_python serial_node.py /dev/ttyACM0``` (If ACM0 doesn't works means that the nucleo is connected to another port)

In the third ```rostopic echo chatter```
