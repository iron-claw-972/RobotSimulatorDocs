# Robot Simulator
How to use the simulator

## Initializing Your Robot
```
from controller import Robot
from controller import Compass
import math
robot = Robot()

timeStep = int(robot.getBasicTimeStep())

leftMotor = robot.getMotor("motor.left") #left wheel if in square path
rightMotor = robot.getMotor("motor.right") #right wheel if in square path

leftEncoder = leftMotor.getPositionSensor()
rightEncoder = rightMotor.getPositionSensor()

leftEncoder.enable(timeStep)
rightEncoder.enable(timeStep)

```
## Initialize Compass (not available on square challenge)
```
compass = robot.getCompass("compass")
compass.enable(timeStep)
def get_bearing_in_degrees():
  north = compass.getValues();
  rad = math.atan2(north[0], north[2])
  bearing = (rad - 1.5708) / math.pi * 180.0
  if (bearing < 0.0):
    bearing = bearing + 360.0
  return bearing
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
target = 0.5  #target is to move at half a wheel rotation every second
leftMotor.setPosition(float('inf')) #allows us to control velocity instead of position
rightMotor.setPosition(float('inf')) #allows us to control velocity instead of position

time=0
while robot.step(timeStep) != -1:
    rightMotor.setVelocity(target*math.pi*2)
    leftMotor.setVelocity(target*math.pi*2)
    print(leftEncoder.getValue(),rightEncoder.getValue()) #print encoder readouts
    time+=1
```

### Turning to an angle

```
speed = 0.3  #target is to move at half a wheel rotation every second
target_angle = 180
leftMotor.setPosition(float('inf')) #allows us to control velocity instead of position
rightMotor.setPosition(float('inf')) #allows us to control velocity instead of position

while robot.step(timeStep) != -1:
    angle = get_bearing_in_degrees()
    if angle>target_angle:
        rightMotor.setVelocity(speed*math.pi*2)
        leftMotor.setVelocity(-speed*math.pi*2)
    if angle<target_angle:
        rightMotor.setVelocity(-speed*math.pi*2)
        leftMotor.setVelocity(speed*math.pi*2)
    if abs(target_angle-angle)<.1:
        print(abs(target_angle-angle))
        rightMotor.setVelocity(0)
        leftMotor.setVelocity(0)
        break
    
```
