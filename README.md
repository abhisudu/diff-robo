# Duct Inspection Robot using Robot Operating System (ROS), ESP8266 Arduino ESP32 CAM and DHT11 Sensor

Heating, ventilation and air conditioning (HVAC) accounts for a significant portion of
buildings to maintain the suitable temperature. For its installation and working, air
conditioning duct are provided. Maintaining the duct clean is a hectic task. Dust particles
accumulate in these regions. Inside the duct condensation of moisture occurs at selective places and it enables the microbial
growth.This robot is developed to inspect such kind of ducts and check for cracks inside ducts and keep track of temperature and humidity inside the duct.

![smartcar2](https://user-images.githubusercontent.com/87858022/131360489-9836ce93-6038-4ffa-9947-860940f4435a.jpg)


## Software:
- [Arduino IDE](https://www.arduino.cc/en/Main/Software)

ESP8266 microcontroller,ESP32 CAM module and DHT11 sensor is programmed using ARDUINO IDE software.

- [Robot Operating System](http://www.ros.org/)

This software is intended to control a 4 wheel differential drive robot via [ROS](https://www.ros.org/) messages using a NodeMcu or ESP8266. The software will respond to [geometry_msgs/Twist](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) messages (precisely __Twist.linear.x__ and __Twist.angular.z__) and generate [PWM](https://en.wikipedia.org/wiki/Pulse-width_modulation) signals accordingly. Programmed with [Arduino](https://www.arduino.cc/) and the [ESP8266 Core](https://github.com/esp8266/Arduino).

## Parts needed:
- Smart car chassis with at least 1 motor on each side, like this: [Smart car chassis 4WD](https://www.aliexpress.com/item/Smart-car-chassis-4WD-4-wheel-drive-force-the-chronological-magnetic-motor-With-code-disc-tachometer/32622219972.html?spm=a2g0s.9042311.0.0.27424c4djmBIqw)
- ESP8266(https://en.wikipedia.org/wiki/NodeMCU) 
- L298N motor driver like this: [L298N](https://www.aliexpress.com/item/L298N-Module-Dual-H-Bridge-Stepper-Motor-Driver-Board-Modules-for-Arduino-Smart-Car-FZ0407-Free/1761850243.html)
- Battery pack or other energy source. 
- Breadboard jumper wires
- ESP32 CAM(AI THINKER)
- DHT11 Sensor
- Servo Motors
- Breadboard jumper wires
- Ubuntu to run ROS
- Wifi access point (or use the access point from the NodeMcu)
- Gamepad (optional)

## Arduino IDE:
- Go to _"Sketch/Include Library/Manage Libraries..."_, search for _"ESP8266"_ and install that library.
- Use the same method to install library _"rosserial"_ or if that doesn't work follow this [tutorial](https://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup).
- If you want to use your own Wifi remove the line containing `#define ACCESS_POINT_SSID "SMARTCAR"` and add your own Wifi credentials into `setupWiFi()`
- Find out the IP address of the computer you intend to use as the ROS serial server and change `IPAddress server(192, 168, 4, 2);` accordingly. (You may need to connect to the `SMARTCAR` Wifi network first if you are using access point mode.)
- If you are using a different micro controller than the NodeMcu v1.0 find out what its highest possible PWM value is and change 'MAX_PWM' and 'MIN_PWN' accordingly. (Max PWM on Arduino is 255, while on a NodeMcu it's 1023)
- Flash _nodemcu-ros-car.ino_ onto the NodeMcu

## ROS Operation:
- Launch rosserial socket node: `roslaunch rosserial_server socket.launch`
- Launch gamepad node in a new terminal: `roslaunch teleop_twist_joy teleop.launch joy_config:=__insert gamepad type__` or 
- Keybord node: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

## Images:


![smartcar](https://user-images.githubusercontent.com/87858022/131361160-416c1d18-9117-4d9a-8bbf-809fe4dfce9d.jpg)
![IMG_20210822_100926](https://user-images.githubusercontent.com/87858022/131361255-5a7493f6-6e1e-4b64-a757-9dd65f160a8b.jpg)
![de](https://user-images.githubusercontent.com/87858022/131361451-e5684e78-7abf-48df-936b-673fadcf6115.JPG)
![IMG_20210830_194311vf3](https://user-images.githubusercontent.com/87858022/131361590-675bcbff-947c-4ded-9e79-4e61cfb5ef7e.jpeg)
![CAM1](https://user-images.githubusercontent.com/87858022/131361687-282676b5-c28e-456b-b6c0-fe778380dfb2.JPG)
![IMG_20210830_194429vf14000](https://user-images.githubusercontent.com/87858022/131361784-dd21e3fe-a97b-419a-9099-06117f70b674.jpeg)
![IMG_20210830_194428vf12000](https://user-images.githubusercontent.com/87858022/131361875-becf54ee-23a1-4fde-ae90-ab683b16ad39.jpeg)
![IMG_20210830_194428vf11000](https://user-images.githubusercontent.com/87858022/131361941-99e84424-43a3-415a-b2f0-2c1062374ce8.jpeg)

#### NodeMcu GPIO pins
![IMG_20210520_133251](https://user-images.githubusercontent.com/87858022/131362147-3b7c9f26-2bd4-4eb0-a63d-5014233a3ff5.JPG)
#### Example L298N motor driver
![698d33db7b9fda12a49cf8cff0fe3120ca4a417e](https://user-images.githubusercontent.com/87858022/131362012-bbe9988a-eff4-4245-87a6-b05e729ead67.png)

