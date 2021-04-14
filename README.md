# Microbit

## Cutebot Docs


### Completely Unrelated Cutebot Tapdancing Code
you can change the beat that it tap dances to in the note_list, and make sure to run it on a hard surface.
```python
music.set_volume(60)
def note(power):
    cuteBot.motors(-power,-power)
    basic.pause(power+20)
    music.ring_tone(power*10)
    cuteBot.motors(power,power)
    basic.pause(power+20)
    cuteBot.motors(0,0)
    while (cuteBot.tracking(cuteBot.TrackingState.L_R_line)):
        basic.pause(3)

note_list = [60,43,60,43,60,43,43,43]


while True:
    for val in note_list:
        note(val)
```

### Mostly Working Random-turning Strategy Code

```python
object_detected = False #stores whether the cutebot has the cube
sonar = 0 #stores distance sensor measurements in centimeters

while True:
    if object_detected:#if the cutebot detects the cube, then drive forward and don't stop.
        cuteBot.motors(70,70) 
    else: #otherwise, either we will be turning if there is a line, or going forward
        sonar = cuteBot.ultrasonic(cuteBot.SonarUnit.CENTIMETERS) #update distance sensor readings
        if cuteBot.tracking(cuteBot.TrackingState.L_R_LINE): #if the cutebot detects a line...
            basic.pause(20) #for 30 milliseconds
            while not cuteBot.tracking(cuteBot.TrackingState.L_R_UNLINE):
                cuteBot.motors(30,-50) #turn cutebot until it is off of the line
            basic.pause(randint(200, 600)) #continue turning for a small random amount of time
            cuteBot.motors(0,0)
        else: #if the cutebot doesn't detect a line...
            cuteBot.motors(50, 50) #go forward and continue searching
        
        if sonar < 20 and sonar > 2: #if the distance is greater than 2cm and less than 20cm then we have the cube
            object_detected = True

    basic.pause(20) #loop delay time 20 milliseconds

```

### Ultrasonic Sensor
```python
distance = cuteBot.ultrasonic(cuteBot.SonarUnit.CENTIMETERS) #gets distance in centimeters
```

### Spinning Motors
```python
cuteBot.motors(100,100) #forward at full speed
cuteBot.motors(0,0) #stop
cuteBot.motors(-100,-100) #backwards at full speed
cuteBot.motors(100,0) #turn right at full speed
cuteBot.motors(0,100) #turn left at full speed
cuteBot.motors(100,-100) #turn right in place at full speed
cuteBot.motors(-100,100) #turn left in place at full speed
```

### Headlights
Mix red, green, and blue to get any color. 255 is the brighest value, 0 means no light
```python
cuteBot.singleheadlights(cuteBot.RGBLights.RGB_L, <red>, <green>, <blue>)
```
Red light:
```python
cuteBot.singleheadlights(cuteBot.RGBLights.RGB_L, 255, 0, 0)
```

Purple light:
```python
cuteBot.singleheadlights(cuteBot.RGBLights.RGB_L, 255, 0, 255)
```

Turn lights off:
```python
cuteBot.singleheadlights(cuteBot.RGBLights.RGB_L, 0, 0, 0)
```

### Buzzer
Play a frequency in Hertz:
```python
music.play_tone(<hertz>, <time in milliseconds>)
```
Play a 3000 Hertz tone for half a second:
```python
music.play_tone(3000, 500)
```

Set volume from 0-255:
```python
music.set_volume(255) #max volume
```

### Light Sensor
There are two light sensors, one of the left and one on the right. These sensors can be used to detect lines by detecting light or dark colored surfaces. When a dark suface is detected, the LEDs above these sensor will light up blue. When a light surface is detected, these LEDs will be off. You can check when the light sensors are activated (when they sense white) with this code. They will be true when white is sensed (except the last one checks when no white is sensed).

```python
bothSensed = cuteBot.tracking(cuteBot.TrackingState.L_R_LINE)
rightSensed = cuteBot.tracking(cuteBot.TrackingState.L_UNLINE_R_LINE)
leftSensed = cuteBot.tracking(cuteBot.TrackingState.L_LINE_R_UNLINE)
noneSensed = cuteBot.tracking(cuteBot.TrackingState.L_R_UNLINE)

```
If you want to have it so the variables are true when it senses black, you can use not:

```python
bothSensed = not cuteBot.tracking(cuteBot.TrackingState.L_R_LINE)
rightSensed = not cuteBot.tracking(cuteBot.TrackingState.L_UNLINE_R_LINE)
leftSensed = not cuteBot.tracking(cuteBot.TrackingState.L_LINE_R_UNLINE)
noneSensed = not cuteBot.tracking(cuteBot.TrackingState.L_R_UNLINE)
```

### How to drive
First, you should use a while True loop. Then you can set your motor values.

```python
while True:
    cuteBot.motors(100,100) #forward at full speed
    basic.pause(70) #wait 1/10th of a second. will keep driving during this time.

```

You can also drive in different ways depending on sensors. In this program, our robot will go forward until it finds an obstacle. When there is something closer than 15cm to it, it will turn in place until it no longer sees it, and then continue driving forward.

```python
while True:
    distance = cuteBot.ultrasonic(cuteBot.SonarUnit.CENTIMETERS) #gets distance in centimeters
    if distance<15:
        cuteBot.motors(-100,100) #turn in place
    else:
        cuteBot.motors(100,100) #go forward
    basic.pause(70) #give the distance sensor a small break. will keep doing what it was doing before during this time

```



## Cutebot Code Structure Code (advanced)

```python

MOVING_FORWARDS = True
MOVING_BACKWARDS = False
STOPPED = False
MOVING_BACKWARDS = False
TURNING_LEFT = False
TURNING_RIGHT = False

DEBRIS_DETECTED = False
LINE_DETECTED = False
BEEPING = False
HEADLIGHTS_ON = False
CURRENT_TIME = input.running_time()
ACTION_START_TIME = 0
ACTION_RUN_TIME = 0

#synchronous actions
TURN_90 = False
MOVE_BACK = False




while True:
    #update sensors
    ##############################
    CURRENT_TIME = input.running_time() #update time
    DEBRIS_DETECTED = cuteBot.ultrasonic(cuteBot.SonarUnit.CENTIMETERS)<10
    LINE_DETECTED = cuteBot.tracking(cuteBot.TrackingState.L_R_UNLINE)# detects white ground

    #updates the state of the robot
    ###############################
    #things that should run all the time
    HEADLIGHTS_ON = LINE_DETECTED #control headlights
    MOVE_BACK = DEBRIS_DETECTED #move back when debris

    #things that run only when an action isn't running
    while (CURRENT_TIME-ACTION_START_TIME)>ACTION_RUN_TIME:
        #default behavior
        #example: if line detected, turn, otherwise go straight.
        #you do not need to mess with the time
        #these are either based on sensor states or are just set here.
        
        MOVING_FORWARDS = True
        MOVING_BACKWARDS = False
        STOPPED = False
        MOVING_BACKWARDS = False
        TURNING_LEFT = False
        TURNING_RIGHT = False

        #synchronous actions
        #these run when the action is triggered
        #they will block the rest of the code in this while loop from running
        #you can set how long they run for with ACTION_RUN_TIME
        #they must being with ACTION_START_TIME = input.running_time()
        #set the action to false when it is done
        #then either trigger another action, or do nothing and return to default
        if MOVE_BACK: 
            ACTION_START_TIME = input.running_time()
            ACTION_RUN_TIME = 500 #goes backwards for half a second
            MOVING_BACKWARDS = True
            MOVING_FORWARDS = False
            MOVE_BACK = False #done moving backwards
            TURN_90 = True #turn 90 degrees after going backwards
            break
        if TURN_90:
            ACTION_START_TIME = input.running_time()
            ACTION_RUN_TIME = 500 #turns for half a second
            MOVING_BACKWARDS = False
            MOVING_FORWARDS = False
            TURNING_LEFT = True
            TURN_90 = False #we are done turning
            break
        break
    


    #runs physical components based on state
    ################################
    if STOPPED: #run motors
        cuteBot.motors(0,0)
    elif TURNING_LEFT:
        cuteBot.motors(0,40)
    elif TURNING_RIGHT:
        cuteBot.motors(40,0)
    elif MOVING_FORWARDS:
        cuteBot.motors(40,40)
    elif MOVING_BACKWARDS:
        cuteBot.motors(-40,-40)

    if HEADLIGHTS_ON: #run headlights
        cuteBot.singleheadlights(cuteBot.RGBLights.RGB_L, 255, 255, 0)
    else:
        cuteBot.singleheadlights(cuteBot.RGBLights.RGB_L, 0, 0, 0)
    
    basic.pause(70)
```


## Cutebot Systems Check Code
```python
# Cutebot Systems Check
# A simple script to make sure that the cutebot is working properly
sonar = 0


# Set motors to max speed
cuteBot.motors(100, 100)
basic.pause(1000)
cuteBot.stopcar()
basic.pause(1000)

# Set motors to max reverse speed
cuteBot.motors(-100, -100)
basic.pause(1000)
cuteBot.stopcar()
basic.pause(1000)

# Turn
cuteBot.motors(0, -50)
basic.pause(1000)
cuteBot.stopcar()

# Check compass
basic.show_arrow(ArrowNames.NORTH)
basic.pause(2000)
basic.clear_screen()

# Check the ultrasonic sensors
while True:
    sonar = cuteBot.ultrasonic(cuteBot.SonarUnit.CENTIMETERS)
    if sonar <= 20:
        basic.show_icon(IconNames.HEART)
    else:
        basic.show_icon(IconNames.SAD)
```


## Snake
```python

points = [[2,2],[2,2],[2,2],[2,2]]
points_length = 4

def limit(number):
    if number<0:
        number=0
    if number>4:
        number=4
    return number


def move(points, right, down):
    new_point = [points[points_length-1][0],points[points_length-1][1]]
    new_point[0] = limit(new_point[0]+right)
    new_point[1] = limit(new_point[1]+down)

    points=points[1:]
    points.append(new_point)
    return points

def clear_grid():
    for y in range(5):
        for x in range(5):
            led.unplot(x, y)

def display(points):
    for point in points:
        led.plot(point[0], point[1])

while True:
    pitch = input.rotation(Rotation.PITCH)
    if pitch<0:
        right=-1
    elif pitch>0:
        right=1
    else:
        right=0

    roll = input.rotation(Rotation.ROLL)
    if roll<0:
        down=-1
    elif roll>0:
        down=1
    else:
        down=0

    points = move(points,down,right)
    clear_grid()
    display(points)
    basic.pause(100)

```


# Robot Simulator
How to use the simulator

## Initializing Your Robot
```python
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
```python
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
## Initialize Distance Sensors
```python
# Get frontal distance sensors.
outerLeftSensor = robot.getDistanceSensor("prox.horizontal.0")
centralLeftSensor = robot.getDistanceSensor("prox.horizontal.1")
centralSensor = robot.getDistanceSensor("prox.horizontal.2")
centralRightSensor = robot.getDistanceSensor("prox.horizontal.3")
outerRightSensor = robot.getDistanceSensor("prox.horizontal.4")

# Enable distance sensors.
outerLeftSensor.enable(timeStep)
centralLeftSensor.enable(timeStep)
centralSensor.enable(timeStep)
centralRightSensor.enable(timeStep)
outerRightSensor.enable(timeStep)
```

## Moving the Robot
### Move the wheels to a position
```python
target = 5  #target is 5 wheel rotations
leftMotor.setPosition(target*math.pi*2) #moves left motor 5 rotations
rightMotor.setPosition(target*math.pi*2)#moves right motor 5 rotations
```

### Move the wheels at a speed
```python
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

```python
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

### Abstraction


We don't want to write that code every time we want to turn to an angle. Instead, we can write a reusable function:
```python
def turn(target_angle, speed, tolerance):
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
      if abs(target_angle-angle)<tolerance:
          print(abs(target_angle-angle))
          rightMotor.setVelocity(0)
          leftMotor.setVelocity(0)
          break
    
```

```python
def move(target_distance, speed, tolerance):
    leftMotor.setPosition(float('inf')) #allows us to control velocity instead of position
    rightMotor.setPosition(float('inf')) #allows us to control velocity instead of position
    while robot.step(timeStep) != -1:
        distance = (leftEncoder.getValue()+rightEncoder.getValue())/2
        if distance<target_distance:
            rightMotor.setVelocity(speed*math.pi*2)
            leftMotor.setVelocity(speed*math.pi*2)
        if distance>target_distance:
            rightMotor.setVelocity(-speed*math.pi*2)
            leftMotor.setVelocity(-speed*math.pi*2)
        if abs(distance-target_distance)<tolerance:
            rightMotor.setVelocity(0)
            leftMotor.setVelocity(0)
            break
            
```

And then when we want to turn, we can call it by typing:

```python
turn(180, 0.3, .1)
```

## Getting Distance Sensor Readings (starts at zero, increases when object is close)
```python
outerLeftSensorValue = outerLeftSensor.getValue() / 360
centralLeftSensorValue = centralLeftSensor.getValue() / 360
centralSensorValue = centralSensor.getValue() / 360
centralRightSensorValue = centralRightSensor.getValue() / 360
outerRightSensorValue = outerRightSensor.getValue() / 360
```

## Simplified Obstacle Avoidance (Part 0).
```python
sensor_conversion_constant = 3600
for segment in range(2000):
    move(segment*.8,.95,.1) #distance speed tolerance
    outerLeftSensorValue = outerLeftSensor.getValue() / sensor_conversion_constant
    centralLeftSensorValue = centralLeftSensor.getValue() / sensor_conversion_constant
    centralSensorValue = centralSensor.getValue() / sensor_conversion_constant
    centralRightSensorValue = centralRightSensor.getValue() / sensor_conversion_constant
    outerRightSensorValue = outerRightSensor.getValue() / sensor_conversion_constant
    if centralSensorValue > 0.1 or outerLeftSensorValue > 0.1 or centralLeftSensorValue > 0.1 or centralRightSensorValue > 0.1 or outerRightSensorValue > 0.1:
        break
 ```
 
## Simplified Obstacle Avoidance (Part 1)
```python
sensor_conversion_constant = 3600
for segment in range(2000):
    move(segment*.8,.95,.1) #distance speed tolerance
    outerLeftSensorValue = outerLeftSensor.getValue() / sensor_conversion_constant
    centralLeftSensorValue = centralLeftSensor.getValue() / sensor_conversion_constant
    centralSensorValue = centralSensor.getValue() / sensor_conversion_constant
    centralRightSensorValue = centralRightSensor.getValue() / sensor_conversion_constant
    outerRightSensorValue = outerRightSensor.getValue() / sensor_conversion_constant
    if centralSensorValue > 0.1 or outerLeftSensorValue > 0.1 or centralLeftSensorValue > 0.1:
        turn(130, 0.95,5)
    if centralRightSensorValue > 0.1 or outerRightSensorValue > 0.1:
        turn(40, 0.95,5)
 ```
 
 
## Full Obstacle Avoidance Example. Not the fastest, can you make a faster one?
 
 ```python
from controller import Robot
from controller import Compass
import math
robot = Robot()
compass = robot.getCompass("compass")

timeStep = int(robot.getBasicTimeStep())

leftMotor = robot.getMotor("motor.left")
rightMotor = robot.getMotor("motor.right")

leftEncoder = leftMotor.getPositionSensor()
rightEncoder = rightMotor.getPositionSensor()

# Get frontal distance sensors.
outerLeftSensor = robot.getDistanceSensor("prox.horizontal.0")
centralLeftSensor = robot.getDistanceSensor("prox.horizontal.1")
centralSensor = robot.getDistanceSensor("prox.horizontal.2")
centralRightSensor = robot.getDistanceSensor("prox.horizontal.3")
outerRightSensor = robot.getDistanceSensor("prox.horizontal.4")

# Enable distance sensors.
outerLeftSensor.enable(timeStep)
centralLeftSensor.enable(timeStep)
centralSensor.enable(timeStep)
centralRightSensor.enable(timeStep)
outerRightSensor.enable(timeStep)

leftEncoder.enable(timeStep)
rightEncoder.enable(timeStep)
compass.enable(timeStep)

def get_bearing_in_degrees():
  north = compass.getValues();
  rad = math.atan2(north[0], north[2])
  bearing = (rad - 1.5708) / math.pi * 180.0
  if (bearing < 0.0):
    bearing = bearing + 360.0
  return bearing


def move(target_distance, speed, tolerance):
    leftMotor.setPosition(float('inf')) #allows us to control velocity instead of position
    rightMotor.setPosition(float('inf')) #allows us to control velocity instead of position
    while robot.step(timeStep) != -1:
        distance = (leftEncoder.getValue()+rightEncoder.getValue())/2
        if distance<target_distance:
            rightMotor.setVelocity(speed*math.pi*2)
            leftMotor.setVelocity(speed*math.pi*2)
        if distance>target_distance:
            rightMotor.setVelocity(-speed*math.pi*2)
            leftMotor.setVelocity(-speed*math.pi*2)
        if abs(distance-target_distance)<tolerance:
            rightMotor.setVelocity(0)
            leftMotor.setVelocity(0)
            break
    
def turn(target_angle, speed, tolerance):
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
      if abs(target_angle-angle)<tolerance:
          rightMotor.setVelocity(0)
          leftMotor.setVelocity(0)
          break
      
      
total_left_sensed = 0
total_right_sensed = 0
rebound_constant = 0.9 #making it higher will make the robot steer away from the obtacle for a longer time
sensor_conversion_constant = 3600
side_responsiveness_constant = .95

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

for segment in range(2000):
    move(segment*.75,.95,.1)
    
    current_angle = (90-get_bearing_in_degrees())
    outerLeftSensorValue = outerLeftSensor.getValue() / sensor_conversion_constant
    centralLeftSensorValue = centralLeftSensor.getValue() / sensor_conversion_constant
    centralSensorValue = centralSensor.getValue() / sensor_conversion_constant
    centralRightSensorValue = centralRightSensor.getValue() / sensor_conversion_constant
    outerRightSensorValue = outerRightSensor.getValue() / sensor_conversion_constant
    
    total_left_sensed = (outerLeftSensorValue*side_responsiveness_constant + centralLeftSensorValue) + rebound_constant * total_left_sensed
    total_right_sensed = (centralRightSensorValue + outerRightSensorValue*side_responsiveness_constant) + rebound_constant * total_right_sensed
    
    if total_left_sensed>total_right_sensed:
        total_left_sensed += centralSensorValue
    else:
        total_right_sensed += centralSensorValue
    
    target_angle = 90+90*clamp(total_left_sensed - total_right_sensed, -1.4, 1.4)    

    turn(target_angle if target_angle>=0 else target_angle+360, .95, 5)


```

