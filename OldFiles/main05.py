# -------------------------------------------------------------
# Imports
# -------------------------------------------------------------
from vex import *
import math
import time

# -------------------------------------------------------------
# Initializations
# -------------------------------------------------------------
# Brain = 004
# Controller = 026 (004 was replaced)
brain=Brain()
controller = Controller()
timer = Timer()

# Motor setup -------------------------------------------------

# From perspective of siting in the robot
#                        Motor(port,         gear ratio,             reverse direction?)

# Left                 = 028  (11)
# Front                = 029  (12)
# Back                 = 026  (13)
# Right                = 025  (14)
left_drive_motor       = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
front_drive_motor      = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
back_drive_motor       = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
right_drive_motor      = Motor(Ports.PORT14, GearSetting.RATIO_18_1, True)

# Right elevator motor = 164  (17) master
# Left elevator motor  = 014  (18) follower
right_elevator_motor   = Motor(Ports.PORT17, GearSetting.RATIO_18_1, False)
left_elevator_motor    = Motor(Ports.PORT18, GearSetting.RATIO_18_1, True)

# End effector motor   = 174  (19)
end_effector_motor     = Motor(Ports.PORT19, GearSetting.RATIO_18_1, True)

# Turns on break mode
left_drive_motor.set_stopping(HOLD)
front_drive_motor.set_stopping(HOLD)
back_drive_motor.set_stopping(HOLD)
right_drive_motor.set_stopping(HOLD)

# Sensor setup ------------------------------------------------

# Inertial sensor = 004 (15)
inertial = Inertial(Ports.PORT15)

# Elevator limit switch = (A)
elevator_limit_switch = Limit(brain.three_wire_port.a)

# AI Vision sensor apriltags = 004 (1)
ai_vision = AiVision(Ports.PORT1, AiVision.ALL_TAGS)

# AI Vision sensor fruit = 000 (2)
ai_vision_2__purple_fruit = Colordesc(1, 106, 83, 143, 10, 0.2)
ai_vision_2__green_fruit = Colordesc(1, 78, 53, 98, 10, 0.2)
ai_vision_2__orange_fruit = Colordesc(1, 167, 113, 193, 10, 0.2)
ai_vision_2 = AiVision(Ports.PORT2, ai_vision_2__purple_fruit)
has_fruit = False

# Ultrasonic sensor left output = (C)
# Ultrasonic sensor left input = (D)
# Ultrasonic sensor right output = (E)
# Ultrasonic sensor right input = (F)
range_finder_left = Sonar(brain.three_wire_port.c)
range_finder_right = Sonar(brain.three_wire_port.e)

# -------------------------------------------------------------
# Variables and constants
# -------------------------------------------------------------
#Robot states
STANDBY       = 0
MOVE          = 1
CALIBRATION   = 2
AUTO          = 3

state = STANDBY # State that the robot starts in

# Seconds - 0 for no auto (current value is 1 hour)
auto_time = 3600

# Auto states
RAMP_START  = 0
RAMP_MID   = 1
RAMP_TOP   = 2

auto_state = RAMP_START # State that the auto starts in

# List of allowed tag IDs (ex. [0, 27, 9])
APRIL_TAG_IDS = [0, 10, 2, 12, 20, 21, 22, 23, 37]
# Set to a tag ID to override list, -1 disables override
APRIL_TAG_SINGLE = -1
current_tag_id = -1
# Center of camera (320 width → 160 center)
CENTER_X = 160
# Ignore outer 40px on each side to help filter messy tag data
EDGE_MARGIN = 20

# Maximum amount the robot can move (30–50 is good)
MAX_STRAFE = 40

# -------------------------------------------------------------
# HELPER FUNCTIONS
# -------------------------------------------------------------
# Stops all motors
def stop_all():
    # print("MOTORS STOPPED")
    left_drive_motor.stop()
    front_drive_motor.stop()
    back_drive_motor.stop()
    right_drive_motor.stop()
    right_elevator_motor.stop()
    left_elevator_motor.stop()
    end_effector_motor.stop()

# Sends all drive motor commands together using speed %
def set_drive(ldp, fdp, bdp, rdp):
    # print("Motor values: ", "ldp ", ldp, "fdp ", fdp, "bdp ", bdp, "rdp ", rdp)
    left_drive_motor.spin(FORWARD, ldp, PERCENT)
    front_drive_motor.spin(FORWARD, fdp, PERCENT)
    back_drive_motor.spin(FORWARD, bdp, PERCENT)
    right_drive_motor.spin(FORWARD, rdp, PERCENT)

# Makes x and y field-oriented instead of robot-oriented using the current heading
def field_oriented(x, y, heading):
    rad = math.radians(heading - 45)
    temp = y * math.cos(rad) + x * math.sin(rad)
    x = -y * math.sin(rad) + x * math.cos(rad)
    y = temp
    return x, y

# Combines strafe_X (left/right), forward/backward, and rotation (spin) into 4 motor speeds for each wheel.
# Then normalizes values to prevent a motor going >100% speed
def x_drive(x, y, rot):
    ldp = y + x + rot
    fdp = y - x - rot
    bdp = y - x + rot
    rdp = y + x - rot
    # Normalize
    max_val = max(abs(ldp), abs(fdp), abs(bdp), abs(rdp), 100)
    return ldp / max_val * 100, fdp / max_val * 100, bdp / max_val * 100, rdp / max_val * 100

# When run, displays info on the brain screen
def display_info():
    # Clear the screen (set it solid black)
    brain.screen.set_fill_color(Color.BLACK)
    brain.screen.draw_rectangle(10, 0, 470, 240)

    # Draw colored state bar (10px wide on left)
    color_map = {STANDBY: Color.GREEN, MOVE: Color.PURPLE, AUTO: Color.BLUE, CALIBRATION: Color.RED}
    brain.screen.set_fill_color(color_map[state])
    brain.screen.draw_rectangle(0, 0, 30, 240)

    mode_text_map = {STANDBY: "STANDBY", MOVE: "DRIVE", CALIBRATION: "CALIBRATION", AUTO: "AUTO"}
    brain.screen.set_cursor(1, 1)
    brain.screen.set_fill_color(color_map[state])
    brain.screen.print(mode_text_map[state])

# runs after the auto time is up
def auto_complete():
    global state
    print("Auto time has run out!")
    state = MOVE

# Zeros the elevator by retracting it all the way down. Then resets the position of the elevator motors
def zero_elevator():
    print("Started zeroing elevator")
    right_drive_motor.set_stopping(COAST)
    left_drive_motor.set_stopping(COAST)
    right_elevator_motor.spin_to_position(360, DEGREES, 100, PERCENT, False)
    left_elevator_motor.spin_to_position(360, DEGREES, 100, PERCENT, True)

    while(elevator_limit_switch.pressing() == False) and (left_elevator_motor.torque(TorqueUnits.NM) < 0.5) and (right_elevator_motor.torque(TorqueUnits.NM) < 0.5):
        print("elevator while zero 179")
        right_elevator_motor.spin(REVERSE, 200)
        left_elevator_motor.spin(REVERSE, 200)
    else:
        right_elevator_motor.stop()
        left_elevator_motor.stop()
        right_elevator_motor.reset_position()
        left_elevator_motor.reset_position()
        print("Finished zeroing elevator")

# Zeros the end effector by opening it all the way open. Then resets the position of the end effector motor
def zero_claw():
    print("Started zeroing end effector")
    while(end_effector_motor.torque(TorqueUnits.NM) < 0.5):
        print("Zee torque: ", end_effector_motor.torque(TorqueUnits.NM))
        end_effector_motor.spin(FORWARD, 50)
    else:
        end_effector_motor.stop()
        end_effector_motor.reset_position()
        print("Finished zeroing end effector")

# Rounds the current heading to a factor of 90 
def get_locked_heading(current_headingl):
    return round(current_headingl / 90) * 90

# Converts width of fruit in camera to the distance the fruit is away from the cam
def camera_distance(width_of_detected_fruit):
    fruit_width_irl = 3.5 # in
    fruit_width_in_cam = 78 # pixals
    fruit_distace_from_cam = 9 # in
    focal_length = (fruit_width_in_cam * fruit_distace_from_cam) / fruit_width_irl
    return ((fruit_width_irl * focal_length) / width_of_detected_fruit)

def get_tilt():
    pitch = inertial.orientation(OrientationType.PITCH, DEGREES)
    roll = inertial.orientation(OrientationType.ROLL, DEGREES)
    return (math.sqrt(pitch**2 + roll**2))

# If a valid tag is found returns [strafe_X, strafe_Y, and rot] needed to be alligned with tag
def allign_to_object():
    print("Aligning to fruit")

    objects_fruit = None
    # objects_purple_fruit = ai_vision_2.take_snapshot(ai_vision_2__purple_fruit)
    # objects_green_fruit = ai_vision_2.take_snapshot(ai_vision_2__green_fruit)
    # objects_orange_fruit = ai_vision_2.take_snapshot(ai_vision_2__orange_fruit)
    objects_fruit = ai_vision_2.take_snapshot(ai_vision_2__purple_fruit)

    strafe_Y = 0
    strafe_X = 0
    rot = 0
    ato_error = True

    target_fruit = None

    target_fruit = objects_fruit[0]
    
    if (target_fruit is not None) and (target_fruit.width > 20):
        error_X = target_fruit.centerX - CENTER_X
        print("fruit", target_fruit.centerX, target_fruit.centerY)

        # Deadzone (prevents jitter) and rotates before moving
        if (abs(error_X) < 10) or (target_fruit.centerX < EDGE_MARGIN or target_fruit.centerX > (320 - EDGE_MARGIN)) or (abs(rot) > 1.5):
            pass
        else:
            # strafe_X = error_X * 0.1
            rot = error_X * 0.1

            if strafe_X > MAX_STRAFE:
                strafe_X = MAX_STRAFE
            elif strafe_X < -MAX_STRAFE:
                strafe_X = -MAX_STRAFE

        TARGET_DISTANCE = 145  # in
        # camera_distance(target_fruit.width)
        error_size = TARGET_DISTANCE - target_fruit.width
        print(target_fruit.width)
        strafe_Y = error_size * 0.3

        if((abs(strafe_X) < 10) and (abs(strafe_Y) < 10) and (abs(rot) < 10)):
            print("s: ", strafe_X, strafe_Y, rot)   
            ato_error = False

    else:
        print("No fruit found")
    
    print("Ato xyre: ", strafe_X, strafe_Y, rot, ato_error)
    return(strafe_X, strafe_Y, rot, ato_error)

# If a valid tag is found returns [strafe_X, strafe_Y, and rot] needed to be alligned with tag
def allign_to_apriltag(tag):
    print("Aligning to tag")
    APRIL_TAG_SINGLE = tag
    objects = ai_vision.take_snapshot(AiVision.ALL_TAGS)

    strafe_Y = 0
    strafe_X = 0
    rot = 0
    atat_error = True

    target_tag = None

    for obj in objects:
        # Single valid tag mode
        if APRIL_TAG_SINGLE != -1:
            if obj.id == APRIL_TAG_SINGLE:
                target_tag = obj
                break

        # Multi valid tags mode
        else:
            if obj.id in APRIL_TAG_IDS:
                target_tag = obj
                break

    # Corrects heading
    current_heading = inertial.heading()
    locked_heading = get_locked_heading(current_heading)

    error_heading = locked_heading - current_heading

    # Wrap error to [-180, 180]
    if error_heading > 180:
        error_heading -= 360
    elif error_heading < -180:
        error_heading += 360

    rot = error_heading * 0.8

    if target_tag is not None:
        error_X = target_tag.centerX - CENTER_X

        # Deadzone (prevents jitter) and rotates before moving
        if (abs(error_X) < 10) or (target_tag.centerX < EDGE_MARGIN or target_tag.centerX > (320 - EDGE_MARGIN)) or (abs(rot) > 1.5):
            strafe_X = 0
        else:
            strafe_X = error_X * 0.3

            if strafe_X > MAX_STRAFE:
                strafe_X = MAX_STRAFE
            elif strafe_X < -MAX_STRAFE:
                strafe_X = -MAX_STRAFE

        TARGET_SIZE = 45  # YOU tune this
        error_size = TARGET_SIZE - target_tag.width
        strafe_Y = error_size * 0.9   # tune this
        print(error_size)

    else:
        print("No tag found")

    APRIL_TAG_SINGLE = -1

    if((strafe_X < 10) and (strafe_Y < 10) and (rot < 10)):
        atat_error = False

    print("AtAT xyr: ", strafe_X, strafe_Y, rot)
    return(strafe_X, strafe_Y, rot, atat_error)

# -------------------------------------------------------------
# MAIN LOOP
# -------------------------------------------------------------
# Runs/loops forever
while True:

    # State transitions ---------------------------------------

    # Changes modes based on butons pressed.
    # Left button = calabration mode
    # Up arrow = move mode
    # Down arrow = auto mode
    # Right button = standby mode
    if controller.buttonLeft.pressing():
        state = CALIBRATION

    elif controller.buttonUp.pressing():
        state = MOVE

    elif controller.buttonDown.pressing():
        state = AUTO

    elif controller.buttonRight.pressing():
        state = STANDBY

    # State logic ---------------------------------------------

    # Standby mode = stops all drive motors -------------------
    if state == STANDBY:
        stop_all()

    # Calabration mode = stops all drive motors and calabrates inertial sensor - ALL IMPUTS ARE BLOCKED DURING CALIBRATION!!!
    elif state == CALIBRATION:
        zero_elevator()
        zero_claw()
        stop_all()
        brain.screen.set_fill_color(Color.RED)
        brain.screen.draw_rectangle(30, 0, 30, 240)
        inertial.calibrate()
        while inertial.is_calibrating():
            wait(20, MSEC)
        brain.screen.set_fill_color(Color.BLACK)
        brain.screen.draw_rectangle(30, 0, 30, 240)
        state = STANDBY

    # Move mode = teleop --------------------------------------
    elif state == MOVE:
        # Elevator logic ------
        if controller.buttonL1.pressing():
            right_elevator_motor.spin(FORWARD, 200)
            left_elevator_motor.spin(FORWARD, 200)
        elif (right_elevator_motor.torque(TorqueUnits.NM) > 0.8) or (left_elevator_motor.torque(TorqueUnits.NM) > 0.8) or (elevator_limit_switch.pressing() == True):
            right_elevator_motor.stop()
            left_elevator_motor.stop()
        elif controller.buttonL2.pressing():
            right_elevator_motor.spin(REVERSE, 200)
            left_elevator_motor.spin(REVERSE, 200)
        else:
            right_elevator_motor.stop()
            left_elevator_motor.stop()
            right_elevator_motor.set_stopping(HOLD)
            left_elevator_motor.set_stopping(HOLD)

        # Claw logic ----------
        if controller.buttonR1.pressing():
            end_effector_motor.spin(FORWARD, 100)

        elif controller.buttonR2.pressing():
            end_effector_motor.spin(REVERSE, 100)

        else:
            end_effector_motor.stop()
            end_effector_motor.set_stopping(HOLD)

        # Auto drive logic ----
        atat_X = 0
        atat_Y = 0
        atat_rot = 0
        atat_error = True
        ato_X = 0
        ato_Y = 0
        ato_rot = 0
        ato_error = True

        # Apriltags -----------
        if controller.buttonX.pressing() or has_fruit == True:
            atat_X, atat_Y, atat_rot, atat_error = allign_to_apriltag(current_tag_id)

            if atat_error == False:
                while(elevator_limit_switch.pressing() == False) and (left_elevator_motor.torque(TorqueUnits.NM) < 0.5) and (right_elevator_motor.torque(TorqueUnits.NM) < 0.5):
                    print("elevator while 421")
                    right_elevator_motor.spin(REVERSE, 200)
                    left_elevator_motor.spin(REVERSE, 200)
                else:
                    right_elevator_motor.stop()
                    left_elevator_motor.stop()

                if (end_effector_motor.torque(TorqueUnits.NM) < 0.5):
                    end_effector_motor.spin(FORWARD, 50)
                else:
                    end_effector_motor.stop()
                    end_effector_motor.set_stopping(HOLD)
                has_fruit = False
            print("has_fruit: ", has_fruit)

        # Fruit ---------------
        if (controller.buttonB.pressing()):
            ato_X, ato_Y, ato_rot, ato_error = allign_to_object()
            if (has_fruit == True):
                ato_X, ato_Y, ato_rot = 0, 0, 0

            if ato_error == False:
                if (end_effector_motor.torque(TorqueUnits.NM) < 0.6):
                    end_effector_motor.spin(REVERSE, 100)
                else:
                    end_effector_motor.stop()
                    end_effector_motor.set_stopping(HOLD)
                    has_fruit = True
            print("has_fruit: ", has_fruit)

        # hfeh_rot = 0
        # if has_fruit == True:
        #     hfeh_rot = (0 - current_heading) * 0.3

        if controller.buttonY.pressing():
            has_fruit = False
            print("has_fruit: ", has_fruit)

        # Drive logic ---------
        Ct_X = controller.axis4.position()
        Ct_Y = controller.axis3.position()
        Ct_rot = controller.axis1.position()

        current_heading = inertial.heading()

        Fo_X, Fo_Y = field_oriented(Ct_X, Ct_Y, current_heading)
        atNfo_X, atNfo_Y = field_oriented(atat_X, atat_Y, 0)
        oNfo_X, oNfo_Y = field_oriented(ato_X, ato_Y, 0)
        x = atNfo_X + oNfo_X + Fo_X
        y = atNfo_Y + oNfo_Y + Fo_Y
        rot = atat_rot + ato_rot + Ct_rot # + hfeh_rot
        # print(Fo_X, Fo_Y, atat_X, atat_Y)
        ldp, fdp, bdp, rdp = x_drive(x, y, rot)
        set_drive(ldp, fdp, bdp, rdp)

    # Auto mode = runs a program and stops after some time ----
    elif state == AUTO:
        if auto_state == RAMP_START:
            print("auto_state: ", auto_state)
            #  Rotate to 45 deg. Move onto ramp
            # When on ramp (using "get_tilt") switch to RAMP_MID

        elif auto_state == RAMP_MID:
            print("auto_state: ", auto_state)
            # Locked rotation at 45 deg. Move up the ramp
            # When finished climbing ramp (using "get_tilt") switch to RAMP_TOP

        elif auto_state == RAMP_TOP:
            print("auto_state: ", auto_state)
            # Switches between finding fruit and finding bins based on if it has a fruit in the claw

        timer.event(auto_complete, auto_time * 1000)
        
    # Update display ------------------------------------------
    # Displays info on the brain screen
    display_info()

    # Program speed limit -------------------------------------
    # Waits for 20 milliseconds [50 times per second (50 Hz)] to prevent CPU overload and flickering, jittering or behaving inconsistently
    wait(20, MSEC)
