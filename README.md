# cansat-spain-2022 :artificial_satellite:
## About the contest
CanSat is an ESA (European Space Agency) initiative for secondary students wich consists on the simulation of a real satellite mission.<br />
![2021-cansat-spain](https://user-images.githubusercontent.com/107350915/173812521-ef8a7ca1-0f5c-4370-ae41-90a1919b3356.png)<br />
Each team builds its own satellite, which must fix inside the dimensions of a normal soda can. The device must meassure temperature, air pressure and barometric altitude every second, and send it by RF to a ground station. Furthermore, each team develops an original secondary mission, and include in their CanSats the necessary gadgets for it. Finally, the devices are thrown with rockets which reach an altitude of 1km, and the descent rate is controlled by a parachute which is included in each satellite.<br />
## Our project
Our team decided to build a CanSat to monitor and prevent volcanic eruptions. For this, we studied different parameters, such as: the air quality (to prevent the population from suffering lung injuries), due to high concentration of carbon dioxide (meassured with an Adafruit CSS811) and PM10 particles (meassured with an optic sensor). Also, volcanic tremor is detected wth an accelerometer, a GPS helps to find and locate our satellite, in order to know where are the values being taken, and the temperature of the lava and the terrain is meassured with a thermal camera.<br />
Last but not least, we developed our own sensor to calculate the magnetic field which forms in the plume, due to the friction of particles, which indicates if the amount of ash which is sent to the atmosphere increases or decreases. It consists of a cooper coil on which is inducted an electric current by the relative movement through the field, which is amplified by two darglinton transistors, and detected by an analog pin on aur microprocessor.<br />
![image](https://user-images.githubusercontent.com/107350915/173815783-be674cd2-c590-4255-960a-e6d42e07a365.png)
![image](https://user-images.githubusercontent.com/107350915/173816424-c4e584e3-244b-4e84-82a7-322f761452cd.png)
## Files
In this repository you will find two files named [satellite.ino](satellite.ino) and [ground_station.ino](ground_station.ino). The first is the main script which runs on the CanSat, and the last is the one of the ground station.
## Schematics
You can find both schematics in the [following picture](schematics.png): cansat (below) and ground station (above), where all components are included.
