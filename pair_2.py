#region VEXcode Generated Robot Configuration
from vex import *
import urandom
import math

# Brain should be defined by default
brain=Brain()

# Robot configuration code


# wait for rotation sensor to fully initialize
wait(30, MSEC)


# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(random))
      
# Set random seed 
initializeRandomSeed()


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

#endregion VEXcode Generated Robot Configuration
# A basic example of commanding the robot to drive forward and backward with the press of a button.

# Library imports
from vex import *

# define the states
IDLE = 0
DRIVING_FWD = 1
DRIVING_BKWD = 2

# start out in the idle state
current_state = IDLE

# Define the brain
brain=Brain()

# Motors
left_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
right_motor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
arm_motor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)

# Controller
controller = Controller()

# Bumper
## TODO: Declare the Bumper here
bumper1 = Bumper(brain.three_wire_port.g)

# Reflectance
## TODO: Declare the reflectance sensor here

# Rangefinder
## TODO: Declare the ultrasonic rangefinder here

"""
Pro-tip: print out state _transistions_.
"""
def handleLeft1Button():
    global current_state

    if(current_state == IDLE):
        print('IDLE -> FORWARD')
        current_state = DRIVING_FWD

        # Note how we set the motor to drive here, just once. 
        # No need to call over and over and over in a loop.
        # Also, note that we call the non-blocking version so we can
        # return to the main loop.

        ## TODO: You'll need to update the speed and number of turns
        left_motor.spin_for(FORWARD, 15.1901639792, TURNS, 91.15, RPM, wait = False)
        right_motor.spin_for(FORWARD, 15.1901639792, TURNS, 91.15, RPM, wait = False)

    else: # in any other state, the button acts as a kill switch
        print(' -> IDLE')
        current_state = IDLE
        left_motor.stop()
        right_motor.stop()

"""
Pro-tip: print out state _transistions_.
"""

    ## Todo: Add code to handle the bumper being presses
    



# Here, we give an example of a proper event checker. It checks for the _event_ 
# of stopping (not just if the robot is stopped).
wasMoving = False
def checkMotionComplete():
    global wasMoving

    retVal = False

    isMoving = left_motor.is_spinning() or right_motor.is_spinning() 

    if(wasMoving and not isMoving):
        retVal = True

    wasMoving = isMoving
    return retVal

# Then we declare a handler for the completion of the motion.
def handleMotionComplete():
    global current_state

    if(current_state == DRIVING_FWD):
        print('FORWARD -> BACKWARD')
        current_state = DRIVING_BKWD

         ## TODO: You'll need to update the speed and number of turns       
        left_motor.spin_for(REVERSE, 15.1901639792, TURNS, 91.15, RPM, wait = False)
        right_motor.spin_for(REVERSE, 15.1901639792, TURNS, 91.15, RPM, wait = False)
    
    elif(current_state == DRIVING_BKWD):
        print('BACKWARD -> IDLE')
        current_state = IDLE

    else:
        print('E-stop') # Should print when button is used as E-stop


## TODO: Add a checker for the reflectance sensor
## See checkMotionComplete() for a good example

## TODO: Add a handler for when the reflectance sensor triggers


"""
The line below makes use of VEX's built-in event management. Basically, we set up a "callback", 
basically, a function that gets called whenever the button is pressed (there's a corresponding
one for released). Whenever the button is pressed, the handleButton function will get called,
_without you having to do anything else_.

"""
controller.buttonL1.pressed(handleLeft1Button)


## TODO: Add event callback for bumper
bumper1.pressed(handleMotionComplete)
"""
Note that the main loop only checks for the completed motion. The button press is handled by 
the VEX event system.
"""

# --- EXTRA CODE ---

REST = 0
MOVE = 1
PICKUP = 2

arm_state = REST
arm_speed = 18

armMoving = False
def checkArmMotionComplete():
    global armMoving

    retVal = False

    isMoving = arm_motor.is_spinning() 

    if(armMoving and not isMoving):
        retVal = True

    armMoving = isMoving
    return retVal

def handleArmMotionComplete():
    global arm_state
    tau = arm_motor.torque(TorqueUnits.NM)

    if(arm_state == MOVE):
        print('MOVE -> PICKUP')
        arm_state = PICKUP
     
        arm_motor.spin_for(FORWARD, 100, DEGREES, 8, RPM, wait = False)
    
    elif(arm_state == PICKUP):
        print('PICKUP -> IDLE')
        print(tau)
        arm_state = IDLE

    else:
        pass



def moveArm():
    global arm_state

    arm_state = MOVE
    arm_motor.spin_for(REVERSE, 875, DEGREES, arm_speed, RPM, wait = False)



def pickup_basket():
    global arm_state
    if arm_state == PICKUP:
        arm_motor.spin_for(FORWARD, 100, DEGREES, arm_speed, RPM, wait = False)
        
    
   #tau = arm_motor.torque(TorqueUnits.NM)
    

controller.buttonA.pressed(moveArm)



# The main loop
while True:
    if(checkMotionComplete()): handleMotionComplete()
    if(checkArmMotionComplete()): handleArmMotionComplete()
    
    
## TODO: Add various checkers/handlers; print ultrasonic; etc. See handout.

        