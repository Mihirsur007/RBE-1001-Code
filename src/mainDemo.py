# Imports =====================================================
from vex import *
import math
import time

# Initializations =============================================
brain=Brain()
controller = Controller() # Controller = 026 (004 was replaced)
timer = Timer()

# Motors From perspective of siting in the robot---------------
left_drive_motor       = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False) # 028
front_drive_motor      = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)  # 029
back_drive_motor       = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False) # 026
right_drive_motor      = Motor(Ports.PORT14, GearSetting.RATIO_18_1, True)  # 025

right_elevator_motor   = Motor(Ports.PORT17, GearSetting.RATIO_18_1, False) # 164
left_elevator_motor    = Motor(Ports.PORT18, GearSetting.RATIO_18_1, True)  # 014

end_effector_motor     = Motor(Ports.PORT19, GearSetting.RATIO_18_1, True)  # 174

# Sensors -----------------------------------------------------
inertial = Inertial(Ports.PORT15)

elevator_limit_switch = Limit(brain.three_wire_port.a)

ai_vision = AiVision(Ports.PORT1, AiVision.ALL_TAGS) # 004 apriltags

ai_vision_2__purple_fruit = Colordesc(1, 151, 125, 195, 40, 1)
ai_vision_2__green_fruit = Colordesc(3, 21, 123, 74, 10, 0.2)
ai_vision_2__orange_fruit = Colordesc(2, 160, 57, 29, 10, 0.2)
ai_vision_2 = AiVision(Ports.PORT2, ai_vision_2__purple_fruit, ai_vision_2__green_fruit, ai_vision_2__orange_fruit) # Fruit

range_finder_left = Sonar(brain.three_wire_port.c) # CD left
range_finder_right = Sonar(brain.three_wire_port.e) # EF right

# Variables and constants =====================================
#States
STANDBY       = 0
MOVE          = 1
CALIBRATION   = 2
AUTO          = 3
state = STANDBY # State that the robot starts in
auto_time = 3600 # How long auto lasts in seconds - "0" for no auto (current value is 1 hour)

APRIL_TAG_IDS = [0, 10, 2, 12, 20, 21, 22, 23, 37] # Tags to look for
APRIL_TAG_SINGLE = -1 # Set to a tag ID to override list, -1 disables override

CENTER_X = 160 # Center of camera (320 width → 160 center)
EDGE_MARGIN = 20 # Ignore outer 40px on each side to help filter messy tag data
MAX_STRAFE = 40 # Maximum amount the robot can move (30–50 is good)

gtb_s1 = False
gtb_gtb_locked_tag = -1
camera_mode = 0 

zc_time = 0
m_searching_direction = True

a_ramp_tilt_mode = 0


# Helper functions ============================================
def display_info(): # Displays info on the brain screen
    # Clear the screen
    brain.screen.set_fill_color(Color.BLACK)
    brain.screen.draw_rectangle(10, 0, 470, 240)
    # Draw colored state bar
    di_color_map = {STANDBY: Color.GREEN, MOVE: Color.PURPLE, AUTO: Color.BLUE, CALIBRATION: Color.RED}
    brain.screen.set_fill_color(di_color_map[state])
    brain.screen.draw_rectangle(0, 0, 30, 240)
    # Draw colored state text
    di_mode_text_map = {STANDBY: "STANDBY", MOVE: "DRIVE", CALIBRATION: "CALIBRATION", AUTO: "AUTO"}
    brain.screen.set_cursor(1, 1)
    brain.screen.set_fill_color(di_color_map[state])
    brain.screen.print(di_mode_text_map[state])
    # print("[display_info]: ", "color: ", di_color_map[state], ", state: ", di_mode_text_map[state])

def stop_all(): # Stops all motors
    left_drive_motor.stop()
    front_drive_motor.stop()
    back_drive_motor.stop()
    right_drive_motor.stop()
    right_elevator_motor.stop()
    left_elevator_motor.stop()
    end_effector_motor.stop()
    print("[stop_all]: MOTORS STOPPED")

def drive_x(x_input, y_input, r_x_input, r_y_input, rotation_input, heading_degrees):
    # > Feild oriented drive, robot oriented drive,       rotation, current heading

    # Converts the current heading in degrees to radians
    heading_rad = math.radians(heading_degrees)

    # Modifys feild oriented drive values to be feild oriented and adds robot oriented drive values
    robot_x = (x_input * math.cos(heading_rad)) - (y_input * math.sin(heading_rad)) + r_x_input
    robot_y = (x_input * math.sin(heading_rad)) + (y_input * math.cos(heading_rad)) + r_y_input

    # Adds rotation values to drive values
    left_drive  = robot_y + rotation_input
    front_drive = -robot_x - rotation_input
    back_drive  = -robot_x + rotation_input
    right_drive = robot_y - rotation_input

    # Makes sure a motor is not going over 100% speed
    max_value = max(abs(left_drive), abs(front_drive), abs(back_drive), abs(right_drive), 100)
    left_drive  = (left_drive / max_value) * 100
    front_drive = (front_drive / max_value) * 100
    back_drive  = (back_drive / max_value) * 100
    right_drive = (right_drive / max_value) * 100

    # Drives wheels
    left_drive_motor.spin(FORWARD, left_drive, PERCENT)
    front_drive_motor.spin(FORWARD, front_drive, PERCENT)
    back_drive_motor.spin(FORWARD, back_drive, PERCENT)
    right_drive_motor.spin(FORWARD, right_drive, PERCENT)
    # print("[drive]:", robot_x, robot_y, r_x_input, r_y_input, left_drive, front_drive, back_drive, right_drive)

def zero_elevator(): # Zeros the elevator by retracting it all the way down. Then resets the position of the elevator motors
    print("[zero_elevator]: started zeroing elevator")
    right_elevator_motor.spin_to_position(360, DEGREES, 50, PERCENT, False)
    left_elevator_motor.spin_to_position(360, DEGREES, 50, PERCENT, True)
    while(elevator_limit_switch.pressing() == False) and (left_elevator_motor.torque(TorqueUnits.NM) < 0.5) and (right_elevator_motor.torque(TorqueUnits.NM) < 0.5):
        print("[zero_elevator]: while loop - ", "elevator_limit_switch pressed? ", elevator_limit_switch.pressing(), ", left_elevator_motor torque: ", left_elevator_motor.torque(TorqueUnits.NM), ", right_elevator_motor torque: ", right_elevator_motor.torque(TorqueUnits.NM))
        right_elevator_motor.spin(REVERSE, 50)
        left_elevator_motor.spin(REVERSE, 50)
    else:
        right_elevator_motor.stop()
        left_elevator_motor.stop()
        right_elevator_motor.reset_position()
        left_elevator_motor.reset_position()
        print("[zero_elevator]: finished zeroing elevator")


def zero_claw(): # Zeros the end effector by opening it all the way open. Then resets the position of the end effector motor
    global zc_time
    print("[zero_claw]: started zeroing end effector")
    # end_effector_motor.spin_to_position(10, DEGREES, 50, PERCENT, True)
    while zc_time < 10:
        end_effector_motor.spin(FORWARD, 100)
        zc_time = zc_time + 1
        wait(20, MSEC)

    while(end_effector_motor.torque(TorqueUnits.NM) < 0.5):
        print("[zero_claw]: while loop - ", "end_effector_motor torque: ", end_effector_motor.torque(TorqueUnits.NM))
        end_effector_motor.spin(FORWARD, 50)
    else:
        end_effector_motor.stop()
        end_effector_motor.reset_position()
        zc_time = 0
        print("[zero_claw]: finished zeroing end effector")

def camera_distance(cd_width_of_detected_fruit): # Converts width of fruit in camera to the distance the fruit is away from the cam
    cd_fruit_width_irl = 3.5 # in
    cd_fruit_width_in_cam = 78 # pixels
    cd_fruit_distace_from_cam = 9 # in
    cd_focal_length = (cd_fruit_width_in_cam * cd_fruit_distace_from_cam) / cd_fruit_width_irl
    print("[camera_distance]: ", "cd_width_of_detected_fruit: ", cd_width_of_detected_fruit, ", camera_distance: ", ((cd_fruit_width_irl * cd_focal_length) / cd_width_of_detected_fruit))
    return ((cd_fruit_width_irl * cd_focal_length) / cd_width_of_detected_fruit)

def get_tilt(): # Returns how tipped the robot is regardless of rotation
    gt_pitch = inertial.orientation(OrientationType.PITCH, DEGREES)
    gt_roll = inertial.orientation(OrientationType.ROLL, DEGREES)
    print("[get_tilt]: ", "gt_pitch: ", gt_pitch, ", gt_roll: ", gt_roll, ", get_tilt: ", (math.sqrt(gt_pitch**2 + gt_roll**2)))
    return (math.sqrt(gt_pitch**2 + gt_roll**2))

def allign_to_object(ato_single_color): # If a valid tag is found returns [strafe_Y, and rot] needed to be alligned with fruit
    print("[allign_to_object]: aligning to fruit")

    ato_objects_fruit = None, None
    ato_purple_objects_fruit = None, None
    ato_green_objects_fruit = None, None
    ato_orange_objects_fruit = None, None

    ato_purple_objects_fruit = ai_vision_2.take_snapshot(ai_vision_2__purple_fruit)
    ato_green_objects_fruit = ai_vision_2.take_snapshot(ai_vision_2__green_fruit)
    ato_orange_objects_fruit = ai_vision_2.take_snapshot(ai_vision_2__orange_fruit)

    ato_strafe_Y = 0
    ato_rot = 0
    ato_complete = False
    ato_target_fruit = None
    ato_target_fruit_min_width = 30
    ato_found_fruit = False
    ato_locked_fruit_color = None

    if (ato_purple_objects_fruit[0].width >= ato_green_objects_fruit[0].width) and (ato_purple_objects_fruit[0].width >= ato_orange_objects_fruit[0].width):
        ato_target_fruit = ato_purple_objects_fruit[0]
        ato_locked_fruit_color = "purple"
    elif (ato_green_objects_fruit[0].width >= ato_purple_objects_fruit[0].width) and (ato_green_objects_fruit[0].width >= ato_orange_objects_fruit[0].width):
        ato_target_fruit = ato_green_objects_fruit[0]
        ato_locked_fruit_color = "green"
    elif (ato_orange_objects_fruit[0].width >= ato_purple_objects_fruit[0].width) and (ato_orange_objects_fruit[0].width >= ato_green_objects_fruit[0].width):
        ato_target_fruit = ato_orange_objects_fruit[0]
        ato_locked_fruit_color = "orange"
    else:
        ato_target_fruit = None

    if (ato_target_fruit is not None) and (ato_target_fruit.width >= ato_target_fruit_min_width):
        ato_found_fruit = True
        print("[allign_to_object]: ato_found_fruit? ", ato_found_fruit, ", number_of_objects: ", len(ato_objects_fruit))
        ato_error_rot = ato_target_fruit.centerX - CENTER_X
        ato_rot = ato_error_rot * 0.15

        if ((not(ato_error_rot > 3)) and (not(ato_error_rot < -3))):
            ato_rot = 0

        ato_TARGET_DISTANCE = 155 # 127
        ato_error_Y = ato_TARGET_DISTANCE - ato_target_fruit.width
        ato_strafe_Y = ato_error_Y * 0.2

        if ((not(ato_strafe_Y > 3)) and (not(ato_strafe_Y < -3))):
            ato_strafe_Y = 0

        if((abs(ato_strafe_Y) < 3) and (abs(ato_rot) < 3)):
            ato_complete = True
    else:
        ato_found_fruit = False
        print("[allign_to_object]: no fruit found")
    print("[allign_to_object]: ", "ato_strafe_X: ", 0, ", ato_strafe_Y: ", ato_strafe_Y, ", ato_rot: ", ato_rot, ", ato_complete? ", ato_complete, ", ato_found_fruit? ", ato_found_fruit, ", ato_locked_fruit_color: ", ato_locked_fruit_color)
    return(0, ato_strafe_Y, ato_rot, ato_complete, ato_found_fruit, ato_locked_fruit_color)

def allign_to_apriltag(atat_tag, atat_full): # If a valid tag is found returns [strafe_X, strafe_Y, and rot] needed to be alligned with tag
    print("[allign_to_apriltag]: aligning to tag")
    APRIL_TAG_SINGLE = atat_tag
    atat_objects = ai_vision.take_snapshot(AiVision.ALL_TAGS)

    atat_strafe_Y = 0
    atat_strafe_X = 0
    atat_rot = 0
    atat_complete = False
    atat_found_tag = False
    locked_tag = -1

    atat_target_tag = None

    for atat_obj in atat_objects:
        # Single valid tag mode
        if APRIL_TAG_SINGLE != -1:
            if atat_obj.id == APRIL_TAG_SINGLE:
                atat_target_tag = atat_obj
                locked_tag = atat_obj.id
                break

        # Multi valid tags mode
        else:
            if atat_obj.id in APRIL_TAG_IDS:
                atat_target_tag = atat_obj
                locked_tag = atat_obj.id
                break

    # Corrects heading
    atat_current_heading = inertial.heading()
    atat_error_heading = 0 - atat_current_heading

    # Wrap error to [-180, 180]
    if atat_error_heading > 180:
        atat_error_heading -= 360
    elif atat_error_heading < -180:
        atat_error_heading += 360

    atat_rot = atat_error_heading * 0.8

    if atat_target_tag is not None:
        atat_found_tag = True
        print("[allign_to_apriltag]: tag found")
        atat_error_X = atat_target_tag.centerX - CENTER_X

        # Deadzone (prevents jitter) and rotates before moving
        if (abs(atat_error_X) < 10) or (atat_target_tag.centerX < EDGE_MARGIN or atat_target_tag.centerX > (320 - EDGE_MARGIN)) or (abs(atat_rot) > 1.5):
            atat_strafe_X = 0
        else:
            atat_strafe_X = atat_error_X * 0.3

            if atat_strafe_X > MAX_STRAFE:
                atat_strafe_X = MAX_STRAFE
            elif atat_strafe_X < -MAX_STRAFE:
                atat_strafe_X = -MAX_STRAFE

        if atat_full == True:
            TARGET_SIZE = 45
        else:
            TARGET_SIZE = 35
        atat_error_size = TARGET_SIZE - atat_target_tag.width
        atat_strafe_Y = atat_error_size * 1.5

    else:
        atat_found_tag = False
        print("[allign_to_apriltag]: no tag found")

    APRIL_TAG_SINGLE = -1

    if((abs(atat_strafe_X) < 10) and (abs(atat_strafe_Y) < 3) and (abs(atat_rot) < 10)):
        atat_complete = True

    print("[allign_to_apriltag]: ", "atat_strafe_X: ", atat_strafe_X, ", atat_strafe_Y: ", atat_strafe_Y, ", atat_rot: ", atat_rot, ", atat_complete? ", atat_complete, ", atat_found_tag? ", atat_found_tag, ", locked_tag: ", locked_tag)
    return(atat_strafe_X, atat_strafe_Y, atat_rot, atat_complete, atat_found_tag, locked_tag)

def auto_complete(): # runs after the auto time is up
    global state
    print("[auto_complete]: auto time has run out!")
    state = MOVE

def rotate(r_angle, r_kp): # Locks to the inputed angle, returns a value to be inputed into "feild oriented"
    r_current_heading = inertial.heading()
    r_error_heading = r_angle - r_current_heading
    r_error_heading = r_error_heading * r_kp
    print("[rotate]: ", "r_angle: ", r_angle, "r_kp: ", r_kp, ", r_error_heading: ", r_error_heading)
    return(r_error_heading)

def go_to_bin(gtb_bin_number):
    print("[go_to_bin]: ", "gtb_bin_number: ", gtb_bin_number)
    global gtb_s1
    global gtb_gtb_locked_tag
    gtb_complete = False
    gtb_atat_strafe_X, gtb_atat_strafe_Y, gtb_atat_rot, gtb_atat_complete0, gtb_atat_found_tag0, gtb_locked_tag0 = allign_to_apriltag(gtb_bin_number, True)
    if gtb_atat_found_tag0 == False:
        if gtb_s1 == False:
            gtb_atat_strafe_X, gtb_atat_strafe_Y, gtb_atat_rot, gtb_atat_complete1, gtb_atat_found_tag1, gtb_locked_tag1 = allign_to_apriltag(-1, False)
            if (gtb_atat_complete1 == True) and (gtb_atat_found_tag1 == True) and (gtb_locked_tag1 != -1):
                gtb_gtb_locked_tag = gtb_locked_tag1
                gtb_s1 = True
        else: # bins left to right: 20-21-22-23
            if gtb_gtb_locked_tag == 20:
                gtb_atat_strafe_X, gtb_atat_strafe_Y, gtb_atat_rot, gtb_atat_complete2, gtb_atat_found_tag2, gtb_locked_tag2 = allign_to_apriltag(gtb_bin_number, True)
                if gtb_atat_found_tag2 == False:
                    if gtb_bin_number == 21:
                        gtb_atat_strafe_X = 20 # ----
                    elif gtb_bin_number == 22:
                        gtb_atat_strafe_X = 20 # ----
                    elif gtb_bin_number == 23:
                        gtb_atat_strafe_X = 20 # ----
                    else:
                        print("spinny boy 20 - weeeeeeeee")
                        controller.rumble('.')
                if (gtb_atat_complete2 == True) and (gtb_atat_found_tag2 == True):
                    gtb_s1 = False
            if gtb_gtb_locked_tag == 21:
                gtb_atat_strafe_X, gtb_atat_strafe_Y, gtb_atat_rot, gtb_atat_complete3, gtb_atat_found_tag3, gtb_locked_tag3 = allign_to_apriltag(gtb_bin_number, True)
                if gtb_atat_found_tag3 == False:
                    if gtb_bin_number == 20:
                        gtb_atat_strafe_X = -20 # ----
                    elif gtb_bin_number == 22:
                        gtb_atat_strafe_X = 20 # ----
                    elif gtb_bin_number == 23:
                        gtb_atat_strafe_X = 20 # ----
                    else:
                        print("spinny boy 21 - weeeeeeeee")
                        controller.rumble('.')
                if (gtb_atat_complete3 == True) and (gtb_atat_found_tag3 == True):
                    gtb_s1 = False
            if gtb_gtb_locked_tag == 22:
                gtb_atat_strafe_X, gtb_atat_strafe_Y, gtb_atat_rot, gtb_atat_complete4, gtb_atat_found_tag4, gtb_locked_tag4 = allign_to_apriltag(gtb_bin_number, True)
                if gtb_atat_found_tag4 == False:
                    if gtb_bin_number == 23:
                        gtb_atat_strafe_X = 20 # ----
                    elif gtb_bin_number == 20:
                        gtb_atat_strafe_X = -20 # ----
                    elif gtb_bin_number == 21:
                        gtb_atat_strafe_X = -20 # ----
                    else:
                        print("spinny boy 22 - weeeeeeeee")
                        controller.rumble('.')
                if (gtb_atat_complete4 == True) and (gtb_atat_found_tag4 == True):
                    gtb_s1 = False
            if gtb_gtb_locked_tag == 23:
                gtb_atat_strafe_X, gtb_atat_strafe_Y, gtb_atat_rot, gtb_atat_complete5, gtb_atat_found_tag5, gtb_locked_tag5 = allign_to_apriltag(gtb_bin_number, True)
                if gtb_atat_found_tag5 == False:
                    if gtb_bin_number == 20:
                        gtb_atat_strafe_X = -20 # ----
                    elif gtb_bin_number == 21:
                        gtb_atat_strafe_X = -20 # ----
                    elif gtb_bin_number == 22:
                        gtb_atat_strafe_X = -20 # ----
                    else:
                        print("spinny boy 23 - weeeeeeeee")
                        controller.rumble('.')
                if (gtb_atat_complete5 == True) and (gtb_atat_found_tag5 == True):
                    gtb_s1 = False
    if (gtb_atat_complete0 == True) and (gtb_atat_found_tag0 == True):
        gtb_complete = True
    print("[go_to_bin]: ", "gtb_s1? ", gtb_s1, ", gtb_atat_found_tag0? ", gtb_atat_found_tag0, ", gtb_gtb_locked_tag: ", gtb_gtb_locked_tag, ", gtb_bin_number: ", gtb_bin_number)
    return (gtb_atat_strafe_X, gtb_atat_strafe_Y, gtb_atat_rot, gtb_complete)

# MAIN LOOP ===================================================
while True:
    # State transitions ---------------------------------------
    if controller.buttonLeft.pressing():    state = CALIBRATION
    elif controller.buttonUp.pressing():    state = MOVE
    elif controller.buttonDown.pressing():  state = AUTO
    elif controller.buttonRight.pressing(): state = STANDBY

    # State logic ---------------------------------------------
    if state == STANDBY:
        stop_all()

    elif state == CALIBRATION: # Calabration mode = stops all drive motors and calabrates inertial sensor - ALL IMPUTS ARE BLOCKED DURING CALIBRATION!!!
        brain.screen.set_fill_color(Color.RED)
        brain.screen.draw_rectangle(30, 0, 30, 240)
        zero_elevator()
        brain.screen.set_fill_color(Color.ORANGE)
        brain.screen.draw_rectangle(30, 0, 30, 240)
        zero_claw()
        zero_claw()
        brain.screen.set_fill_color(Color.YELLOW)
        brain.screen.draw_rectangle(30, 0, 30, 240)
        stop_all()
        brain.screen.set_fill_color(Color.BLUE)
        brain.screen.draw_rectangle(30, 0, 30, 240)
        inertial.calibrate()
        while inertial.is_calibrating():
            wait(20, MSEC)
        brain.screen.set_fill_color(Color.BLACK)
        brain.screen.draw_rectangle(30, 0, 30, 240)
        state = MOVE

    elif state == MOVE: # teleop
        # Elevator logic ------
        if controller.buttonL1.pressing():
            right_elevator_motor.spin(FORWARD, 100)
            left_elevator_motor.spin(FORWARD, 100)
        elif controller.buttonL2.pressing():
            right_elevator_motor.spin(REVERSE, 100)
            left_elevator_motor.spin(REVERSE, 100)
        else:
            right_elevator_motor.stop()
            left_elevator_motor.stop()

        # Claw logic ----------
        if controller.buttonR1.pressing():
            end_effector_motor.spin(FORWARD, 100)

        elif controller.buttonR2.pressing():
            end_effector_motor.spin(REVERSE, 100)

        else:
            end_effector_motor.stop()
            end_effector_motor.set_stopping(HOLD)

        # Apriltags -----------
        m_atat_strafe_X = 0
        m_atat_strafe_Y = 0
        m_atat_rot = 0

        if controller.buttonX.pressing():
            m_atat_strafe_X, m_atat_strafe_Y, m_atat_rot, m_atat_complete, m_atat_found_tag, gtb_locked_tag = allign_to_apriltag(-1, True)
            if m_atat_found_tag == False:
                controller.rumble('-')

        # Drive logic ---------
        m_c_X = -controller.axis4.position()
        m_c_Y = -controller.axis3.position()
        m_c_rot = controller.axis1.position()

        current_heading = inertial.heading()

        drive_x(m_c_X, m_c_Y, m_atat_strafe_X, m_atat_strafe_Y, (m_c_rot + m_atat_rot), current_heading)

        # print("[m: ]", m_ato_strafe_X, m_ato_strafe_Y, m_ato_rot, m_ato_complete, m_ato_found_fruit, "gtb", m_gtb_atat_strafe_X, m_gtb_atat_strafe_Y, m_gtb_atat_rot, "m: ", m_ato_ro_X, m_ato_ro_Y)

    # Auto mode = runs a program and stops after some time ----
    elif state == AUTO:

        m_gtb_atat_strafe_X, m_gtb_atat_strafe_Y, m_gtb_atat_rot = 0, 0, 0
        m_ato_strafe_X, m_ato_strafe_Y, m_ato_rot, m_ato_complete, m_ato_found_fruit = 0, 0, 0, False, False
        m_color_to_bin_id = 20
        m_the_color = None
   
        a_gtb_atat_strafe_X = 0
        a_gtb_atat_strafe_Y = 0
        a_gtb_atat_rot = 0
        a_gtb_complete = False

        if controller.buttonB.pressing():
            a_ramp_tilt_mode = 3
        if controller.buttonA.pressing():
            a_ramp_tilt_mode = 4
        if controller.buttonX.pressing():
            a_ramp_tilt_mode = 0

        m_c_X = -controller.axis4.position()
        m_c_Y = -controller.axis3.position()
        m_c_rot = controller.axis1.position()


        if a_ramp_tilt_mode == 3:
            if camera_mode == 0:
                m_ato_strafe_X, m_ato_strafe_Y, m_ato_rot, m_ato_complete, m_ato_found_fruit, m_target_fruit = allign_to_object(-1)
                # if (m_target_fruit != None) and (m_target_fruit != -1):
                m_the_color = m_target_fruit
                if m_ato_found_fruit == False:
                    # controller.rumble('.')
                    if inertial.heading() <= 90:
                        m_searching_direction = True
                    if inertial.heading() >= 270:
                        m_searching_direction = False
                    if m_searching_direction == True:
                        m_ato_rot = 10
                    else:
                        m_ato_rot = -10
                if (m_ato_complete == True) and (m_ato_found_fruit ==True):
                    print("abcde")
                    stop_all()
                    end_effector_motor.spin(REVERSE, 50)
                    wait(1000, MSEC)
                    camera_mode = 1
            if camera_mode == 1:
                end_effector_motor.spin(REVERSE, 100)
                if end_effector_motor.torque(TorqueUnits.NM) > 0.5:
                    end_effector_motor.set_stopping(HOLD)
                    camera_mode = 2
            if camera_mode == 2:
                if m_target_fruit == "purple": # 23
                    m_color_to_bin_id = 23
                    brain.screen.set_fill_color(Color.PURPLE)
                    brain.screen.draw_rectangle(60, 0, 60, 240)
                elif m_target_fruit == "green": # 22
                    m_color_to_bin_id = 22
                    brain.screen.set_fill_color(Color.GREEN)
                    brain.screen.draw_rectangle(60, 0, 60, 240)
                elif m_target_fruit == "orange": # 20
                    m_color_to_bin_id = 20
                    brain.screen.set_fill_color(Color.ORANGE)
                    brain.screen.draw_rectangle(60, 0, 60, 240)
                else:
                    print("noooooo color")
                    brain.screen.set_fill_color(Color.RED)
                    brain.screen.draw_rectangle(60, 0, 60, 240)
                m_gtb_atat_strafe_X, m_gtb_atat_strafe_Y, m_gtb_atat_rot, m_gtb_complete = go_to_bin(m_color_to_bin_id)
                print("hdjfsdhkjfhskjdhf: ", m_color_to_bin_id, m_the_color, m_target_fruit)
                if m_gtb_complete == True:
                    print("zyxw")
                    stop_all()
                    end_effector_motor.set_stopping(COAST)
                    end_effector_motor.spin(FORWARD, 50)
                    wait(1000, MSEC)
                    camera_mode = 3
            if camera_mode == 3:
                m_gtb_atat_rot = 180 - inertial.heading()
                if abs(m_gtb_atat_rot) < 1:
                    camera_mode = 0
            print("camera mode: ", camera_mode)
        else:
            camera_mode = 0

        m_a_rtm_5_rot = 0
        if a_ramp_tilt_mode == 5:
            m_a_rtm_5_rot = 0 - inertial.heading()
            if m_a_rtm_5_rot < 3:
                a_ramp_tilt_mode = 6

        if a_ramp_tilt_mode == 6:
            a_rtm_strafe_X, a_rtm_strafe_Y, a_rtm_rot, a_rtm_complete, a_rtm_found_tag, a_rtm_locked_tag = allign_to_apriltag(-1, False)
            if (((a_rtm_complete == True) and (a_rtm_found_tag == True)) or (a_rtm_found_tag == False)):
                a_ramp_tilt_mode = 3

        a_ramp_Y = 0
        a_ramp_rot = 0
        if controller.buttonY.pressing():
            # use get_tilt() to determine angle and switch states when flat
            if(get_tilt() < 10) and ((a_ramp_tilt_mode == 0) or (a_ramp_tilt_mode == 2)):
                if a_ramp_tilt_mode == 2:
                   a_ramp_tilt_mode = 5
                else:
                    a_ramp_rot = rotate(135, 0.5)
                    a_ramp_Y = -20
                
            if(get_tilt() >= 10) and (a_ramp_tilt_mode != 4):
                if a_ramp_tilt_mode != 2:
                    a_ramp_tilt_mode = 1
                a_ramp_rot = rotate(180, 0.5)
                a_ramp_Y = -100

            if a_ramp_tilt_mode == 1:
                a_ramp_tilt_mode = 2
                a_ramp_rot = 0
                a_ramp_Y = -40
                wait(3000, MSEC)

        current_heading = inertial.heading()

        m_rot = a_ramp_rot + m_ato_rot + m_gtb_atat_rot + m_a_rtm_5_rot

        drive_x(m_c_X, (a_ramp_Y + m_c_Y), (m_ato_strafe_X + m_gtb_atat_strafe_X + a_gtb_atat_strafe_X), (m_ato_strafe_Y + m_gtb_atat_strafe_Y + a_gtb_atat_strafe_Y), (a_ramp_rot + m_ato_rot + m_gtb_atat_rot + m_c_rot), current_heading)

        timer.event(auto_complete, auto_time * 1000)
        
    # Update display ------------------------------------------
    # Displays info on the brain screen
    display_info()

    # Program speed limit -------------------------------------
    # Waits for 20 milliseconds [50 times per second (50 Hz)] to prevent CPU overload and flickering, jittering or behaving inconsistently
    wait(80, MSEC)
