# Robot Simulator
How to use the simulator

## Initializing Stuff
```
from controller import Robot
import math
robot = Robot()
```

## Moving the Robot
### Move the wheels to a position
```
target = 10 #target is 10 millimeters
robot.getMotor("motor.left").setPosition(target/21) #moves left motor 10mm
robot.getMotor("motor.right").setPosition(target/21)#moves right motor 10mm
```
