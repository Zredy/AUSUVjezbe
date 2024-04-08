# AUSUVjezbe - ascuric

Zadaća 6:

U sklopu zadaće potrebn je napraviti:
- urdf datoteku KUKA LBR IIWA 14 robota
- napraviti čvor state_publisher koji objavljuje na temu /joint_states, kuteve svih 7 linkova od -60 stupnjeva do 60 stupnjeva.

  
Zadaću predati kao video gibanja robota.

Naredbe za preuzimanje repozitorija i pokretanje:
```
mkdir ~/zadaca6_ws
cd ~/zadaca6_ws
git clone -b Zadaca6 --single-branch https://github.com/Zredy/AUSUVjezbe.git .
colcon build
source install/setup.bash
ros2 launch rsp_tutorial iiwa14.launch.py
```
