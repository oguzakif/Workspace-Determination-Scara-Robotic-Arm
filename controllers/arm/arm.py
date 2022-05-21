
from controller import Robot
from controller import Supervisor

def checkCollision(sensor):
    return sensor.getValue() < 400
  
# create the Robot instance.

robot = Supervisor()

root_node = robot.getRoot()
children_field = root_node.getField('children')

# get the time step of the current world.
timestep = 500

armPosition = 0.0
endEffectorPosition = 0.0

armMotor = robot.getDevice('armMotor')
armMotor.setPosition(armPosition)

headMotor = robot.getDevice('headMotor')
headMotor.setPosition(endEffectorPosition)

distanceSensor1 = robot.getDevice("operatorSensor1")
distanceSensor1.enable(timestep)

distanceSensor2 = robot.getDevice("operatorSensor2")
distanceSensor2.enable(timestep)

endEffectorGPS = robot.getDevice("gps")
endEffectorGPS.enable(timestep)
print(endEffectorGPS)

stepVal = 0.2
clockwiseFlag = False
armFlag = False
counter = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    if(checkCollision(distanceSensor1)):
        #print("Collision on sensor 1!")      
        clockwiseFlag = False  
        armFlag = True
      
    elif(checkCollision(distanceSensor2)):
        #print("Collision on sensor 2!")
        clockwiseFlag = True
    
    if(endEffectorPosition > headMotor.getMaxPosition()):
        clockwiseFlag = True
    elif(endEffectorPosition < headMotor.getMinPosition()):
        clockwiseFlag = False
        armFlag = True
    
    print("flag -> ", clockwiseFlag)
    print("headMotorPosition -> ",endEffectorPosition)
    print("end effector coords -> ", endEffectorGPS.getValues())
    
    endEffectorPosition += -stepVal if clockwiseFlag else stepVal
    headMotor.setPosition(endEffectorPosition)
    children_field.importMFNodeFromString(-1, "DEF pointer_node"+str(counter)+" Solid {children [ Shape { appearance PBRAppearance { baseColor 0 1 0 transparency 0.8 roughness 1 metalness 0 } geometry Box { size 0.01 0.01 0.01 } castShadows FALSE }]}")
    point = robot.getFromDef("pointer_node"+str(counter))
    translation_field = point.getField('translation')
    translation_field.setSFVec3f(endEffectorGPS.getValues())
    if(armFlag):
        armPosition += stepVal
        armMotor.setPosition(armPosition)
        armFlag = False
        
    counter += 1
    pass

# Enter here exit cleanup code.
