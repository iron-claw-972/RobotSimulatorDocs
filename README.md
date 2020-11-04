# Robot Simulator
How to use the simulator

## Initializing Your Robot
```
from controller import Robot
from controller import Compass
import math
robot = Robot()
timeStep = int(robot.getBasicTimeStep())

leftMotor = robot.getMotor("motor.left")
rightMotor = robot.getMotor("motor.right")

leftEncoder = leftMotor.getPositionSensor()
rightEncoder = rightMotor.getPositionSensor()

leftEncoder.enable(100)
rightEncoder.enable(100)


```

## Moving the Robot
### Move the wheels to a position
```
target = 5  #target is 5 wheel rotations
leftMotor.setPosition(target*math.pi*2) #moves left motor 5 rotations
rightMotor.setPosition(target*math.pi*2)#moves right motor 5 rotations
```

### Move the wheels at a speed
```
target = 0.5  #target is to move at half a wheel rotations every second
leftMotor.setPosition(float('inf')) #allows us to control velocity instead of position
rightMotor.setPosition(float('inf')) #allows us to control velocity instead of position

while robot.step(timeStep) != -1:
    rightMotor.setVelocity(target*math.pi*2)
    leftMotor.setVelocity(target*math.pi*2)
```


