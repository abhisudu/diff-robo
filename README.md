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

## TESTING:

https://user-images.githubusercontent.com/87858022/132352627-1d375429-1634-4990-b3b7-7c013f89dc3d.mp4


https://user-images.githubusercontent.com/87858022/131982615-3385f40a-8e6b-4e2f-9e42-2996aba923aa.mp4


https://user-images.githubusercontent.com/87858022/131789231-de1599ea-d95f-4999-9b7b-e5fe2fd2ac0b.mp4


https://user-images.githubusercontent.com/87858022/131789517-02038d64-7ec8-4a17-9a38-d34f4387e917.mp4


https://user-images.githubusercontent.com/87858022/131789568-68c845e2-92f3-4a1e-9c8e-2de5ca565a91.mp4


https://user-images.githubusercontent.com/87858022/131789585-2f569d23-69dc-41ea-967b-498b6a0afce2.mp4


https://user-images.githubusercontent.com/87858022/131789889-70af0ae5-3c8d-4795-be36-b1a599462187.mp4

https://user-images.githubusercontent.com/87858022/131790104-820b30db-713f-4dc5-9485-c1926acdf722.mp4

## Images:


![smartcar](https://user-images.githubusercontent.com/87858022/131361160-416c1d18-9117-4d9a-8bbf-809fe4dfce9d.jpg)
![Screenshot from 2021-08-31 10-53-05](https://user-images.githubusercontent.com/87858022/131449467-cfa9a9df-0639-433b-b83c-c402d11b0630.png)
![IMG_20210822_100926](https://user-images.githubusercontent.com/87858022/131361255-5a7493f6-6e1e-4b64-a757-9dd65f160a8b.jpg)
![IMG_20210907_204114](https://user-images.githubusercontent.com/87858022/132369947-f6792f5a-ec93-408f-8238-7ba9bfd12859.jpg)
![PicsArt_09-07-08 47 29](https://user-images.githubusercontent.com/87858022/132370199-63d9991e-55ec-46e7-8ba7-6d3d7f801a0a.jpg)
![de](https://user-images.githubusercontent.com/87858022/131361451-e5684e78-7abf-48df-936b-673fadcf6115.JPG)
![IMG_20210830_194311vf3](https://user-images.githubusercontent.com/87858022/131361590-675bcbff-947c-4ded-9e79-4e61cfb5ef7e.jpeg)
![CAM1](https://user-images.githubusercontent.com/87858022/131361687-282676b5-c28e-456b-b6c0-fe778380dfb2.JPG)
![IMG_20210830_194429vf14000](https://user-images.githubusercontent.com/87858022/131361784-dd21e3fe-a97b-419a-9099-06117f70b674.jpeg)
![IMG_20210830_194428vf12000](https://user-images.githubusercontent.com/87858022/131361875-becf54ee-23a1-4fde-ae90-ab683b16ad39.jpeg)
![IMG_20210830_194428vf11000](https://user-images.githubusercontent.com/87858022/131361941-99e84424-43a3-415a-b2f0-2c1062374ce8.jpeg)
![rosgraph](https://user-images.githubusercontent.com/87858022/132350448-50bb691b-6716-4ecd-bb0b-26674d938b01.png)
#### NodeMcu GPIO pins
![IMG_20210520_133251](https://user-images.githubusercontent.com/87858022/131362147-3b7c9f26-2bd4-4eb0-a63d-5014233a3ff5.JPG)
#### Example L298N motor driver
![mod](https://user-images.githubusercontent.com/87858022/131449612-f58e2409-3165-483e-b834-8c4ae7c3fbf6.JPG)

#### DHT11
![DHT11-Module-Pinout (1)](https://user-images.githubusercontent.com/87858022/131793061-37df9102-493a-4785-8a6e-5b2f46e0e20f.png)

#### ESP32CAM
![ksS4E](https://user-images.githubusercontent.com/87858022/131793266-1ed0e2bd-5f22-49a9-87ab-6f5769e7272d.png)
