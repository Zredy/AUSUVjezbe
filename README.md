# AUSUVjezbe - ascuric

Zadaća 6:

U sklopu zadaće potrebn je napraviti:
- urdf datoteku KUKA LBR IIWA 14 robota
- napraviti čvor state_publisher koji objavljuje na temu /joint_states, kuteve svih 7 linkova od -60 stupnjeva do 60 stupnjeva.

  
Zadaću predati kao video gibanja robota.

Preuzeti repozitorij kao .zip i unzip napraviti u novi folder, ili folder ~/rsp_ws

Napraviti naredbe:
```
cd ~/rsp_ws
colcon build
source install/setup.bash
ros2 launch rsp_tutorial iiwa14.launch.py
```
