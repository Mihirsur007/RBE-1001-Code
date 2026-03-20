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
# The main loop
while True:
    if(checkMotionComplete()): handleMotionComplete()

## TODO: Add various checkers/handlers; print ultrasonic; etc. See handout.

        
