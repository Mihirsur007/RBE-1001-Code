from vex import *
import time
# define the states
IDLE = 0
DOWN = 1
UP = 2

# start out in the idle state
current_state = IDLE

# variable to check if it was moving
was_moving = False

# Define the brain
brain=Brain()

#Basket
basket_weight = 100
#Motors
arm_motor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
left_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
right_motor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)

#Controller
controller = Controller()

#Drivetrain
drivetrain = DriveTrain(left_motor, right_motor)

# distance sensor
distance_sensor = Sonar(brain.three_wire_port.d)

def arm_down():
    drivetrain.stop()
    global current_state

    if(current_state == IDLE):
        print('IDLE -> FORWARD')
        current_state = DOWN
    else: # in any other state, the button acts as a kill switch
        print(' -> IDLE')
        current_state = IDLE
        arm_motor.stop()
    
    arm_motor.spin_for(REVERSE, 880, DEGREES, 20, RPM, True)
    
def picked_up():
    if (arm_motor.torque(TorqueUnits.NM)) == ():
        print("Basket Picked Up")  
    else:
        print("Basket Not Picked Up")


def checkArmMotionComplete():
    global was_moving

    ret_val = False

    is_moving = arm_motor.is_spinning()

    if(was_moving and not is_moving):
        ret_val = True

    was_moving = is_moving
    return ret_val

def handle_arm_motion_complete():
    global current_state
    if current_state == DOWN:
        print("DOWN-> UP")
        current_state = UP
        time.sleep(3)
        drivetrain.drive_for(REVERSE, 3.5, INCHES)
        arm_motor.spin_for(FORWARD, 100, DEGREES, 20, RPM, True)
        print(arm_motor.torque(TorqueUnits.NM))
        #picked_up()

    elif(current_state == UP):
        print('UP -> IDLE')
        current_state = IDLE

    else:
        print('E-stop') # Should print when button is used as E-stop




def move_forward():
    
    while distance_sensor.distance(INCHES) > 12.5:
        drivetrain.drive(REVERSE)
    arm_down()
    drivetrain.stop()


controller.buttonL2.pressed(move_forward)

while True:
    print(distance_sensor.distance(INCHES))
    if (checkArmMotionComplete()): handle_arm_motion_complete()
    
