
from controller import Robot

def checkCollision(sensor):
    print(sensor.getValue())
    return sensor.getValue() < 350
  
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
#timestep = int(robot.getBasicTimeStep())
timestep = 64

armMotor = robot.getDevice('armMotor')
armMotor.setPosition(float("inf"))
armMotor.setVelocity(0.0)


headMotor = robot.getDevice('headMotor')
headMotor.setPosition(float("inf"))
headMotor.setVelocity(0.0)

speedArm = 1
speedHead = 1
pSensor = robot.getDevice("armPositionSensor")
pSensor.enable(timestep)

distanceSensor1 = robot.getDevice("operatorSensor1")
distanceSensor1.enable(timestep)

print(distanceSensor1)

distanceSensor2 = robot.getDevice("operatorSensor2")
distanceSensor2.enable(timestep)

print(distanceSensor2)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    armMotor.setVelocity(speedArm)
    headMotor.setVelocity(speedHead)
    
    if(checkCollision(distanceSensor1) or checkCollision(distanceSensor2)):
        speedHead = -speedHead
        speedArm = -speedArm
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #val = pSensor.getValue()
    #print(val)
    # Process sensor data here.

    pass

# Enter here exit cleanup code.
