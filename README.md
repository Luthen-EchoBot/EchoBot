# GeiCar Project

The GeiCar project is a project carried out by students at [INSA Toulouse](http://www.insa-toulouse.fr/fr/index.html). This project consists in developing the software of a autonomous car in order to carry out different missions. Several projects are exposed on the [official website] (https://sites.google.com/site/projetsecinsa/).

This repository is intended to provide a basis for students starting a new project on the GeiCar. The present code as well as the documentation is the result of internship carried out by [Alexis24](https://github.com/Alexix24) (Alexis Pierre Dit Lambert)

The platform is developped and maintained by :

* DI MERCURIO Sébastien
* LOMBARD Emmanuel
* MARTIN José

The projects are (or were) surpervised by:

* CHANTHERY Elodie
* DELAUTIER Sébastien
* MONTEIL Thierry
* LE BOTLAN Didier
* AURIOL Guillaume
* DI MERCURIO Sébastien

The projects are done by the following students:
* Grégoire Drénou
* Ugo Teuliere
* Victor Lasserre
* Marvin Ishq Yonan
* Dimitri Pizzinat
* David Cocoma

## Quick User Guide
### Turn the car on and off
* To turn on the car:
  * Toggle the red button to bring the power.
  * Press the START push button (hold it down for a short while).
  * Wait until Raspberry boot up and connect to it using its IP address (written on the board): `ssh pi@10.105.1.xx`
  * When connected start ROS subsystem using :`ros2 launch geicar_start geicar.launch.py`
  * Then, you will get a report of subsystems and be able to control the car using XBOX controler 

* To turn off the car:
	* Use the red button as a switch to turn off the power.

### Use of this repository
* First of all, [fork this repository](https://docs.github.com/en/get-started/quickstart/fork-a-repo) to get yours.
* Then, clone your fresh and new repository on your computer: `git clone https://github.com/<your id>/<your wonderful repo>`
* Have a look in "general" directory for how to connect and work with your car
* For more 'in depth' documentation on a particular subsystem, have a look in following directories:
    * raspberryPI3: Everything about raspberry setup
    * nucleoL476: Stuff about GPS-RTK and IMU
    * nucleoF103: informations on motors control, main power and ultrasonics sensors
    * jetsonNano: Directory containing info on IA, Camera and Lidar
    * simulation: if you want to setup a carla simulation environment

__warning__
You normally do not need to change firmware running in F103 and F476 boards. You main work is on the raspberry and jetson side.

## First commands for car control

### Terminal 1
* Connect to the Raspberry Pi
```
ssh pi@10.105.1.167
```
password  ``` geicar  ```

* Launch ros2
```
ros2 launch geicar_start geicar.launch.py
 ```
### Terminal 2
* Connect to the Raspberry Pi
```
ssh pi@10.105.1.167
```
password  ``` geicar  ```

* Connect to the Jetson 
 ```
goto_jetson
 ```

This previous command is an alias equivalent to the command “sshpass -p jetson ssh jetson@192.168.1.10”

*Launch the Docker
``` 
sudo docker start -ai ros-humble
```

* Launch the Camera and the Lidar
```
ros2 launch geicar_start_jetson geicar.jetson.launch.py
 ```


