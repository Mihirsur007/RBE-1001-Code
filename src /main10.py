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
# Auto states
RAMP_START    = 0
RAMP_MID      = 1
RAMP_TOP      = 2
auto_state = RAMP_START # State that the auto starts in
auto_time = 3600 # How long auto lasts in seconds - "0" for no auto (current value is 1 hour)

APRIL_TAG_IDS = [0, 10, 2, 12, 20, 21, 22, 23, 37] # Tags to look for
APRIL_TAG_SINGLE = -1 # Set to a tag ID to override list, -1 disables override
current_tag_id = -1

CENTER_X = 160 # Center of camera (320 width → 160 center)
EDGE_MARGIN = 20 # Ignore outer 40px on each side to help filter messy tag data
MAX_STRAFE = 40 # Maximum amount the robot can move (30–50 is good)

gtb_s1 = False
gtb_gtb_locked_tag = -1
a_bin = -1

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

def set_drive(sd_ldp, sd_fdp, sd_bdp, sd_rdp): # Sends all drive motor commands together using speed %
    left_drive_motor.spin(FORWARD, sd_ldp, PERCENT)
    front_drive_motor.spin(FORWARD, sd_fdp, PERCENT)
    back_drive_motor.spin(FORWARD, sd_bdp, PERCENT)
    right_drive_motor.spin(FORWARD, sd_rdp, PERCENT)
    # print("[set_drive]: ", "ldp: ", sd_ldp, ", fdp: ", sd_fdp, ", bdp: ", sd_bdp, ", rdp: ", sd_rdp)

def field_oriented(fo_x, fo_y, fo_heading): # Makes x and y field-oriented instead of robot-oriented using the current heading
    fo_rad = math.radians(fo_heading - 45)
    fo_temp = fo_y * math.cos(fo_rad) + fo_x * math.sin(fo_rad)
    fo_xo = -fo_y * math.sin(fo_rad) + fo_x * math.cos(fo_rad)
    fo_yo = fo_temp
    # print("[field_oriented]: ", "fo_x: ", fo_x, ", fo_y: ", fo_y, ", fo_heading: ", fo_heading, ", fo_xo: ", fo_xo, ", fo_yo: ", fo_yo)
    return fo_xo, fo_yo

def x_drive(xd_x, xd_y, xd_rot): # Combines movement and rotation into % speeds for each wheel. Normalizes values to prevent a motor going >100% speed
    xd_ldp = xd_y + xd_x + xd_rot
    xd_fdp = xd_y - xd_x - xd_rot
    xd_bdp = xd_y - xd_x + xd_rot
    xd_rdp = xd_y + xd_x - xd_rot
    xd_max_val = max(abs(xd_ldp), abs(xd_fdp), abs(xd_bdp), abs(xd_rdp), 100)
    xd_ldpo = xd_ldp / xd_max_val * 100
    xd_fdpo = xd_fdp / xd_max_val * 100
    xd_bdpo = xd_bdp / xd_max_val * 100
    xd_rdpo = xd_rdp / xd_max_val * 100
    # print("[x_drive]: ", "xd_x: ", xd_x, ", xd_y: ", xd_y, ", xd_rot: ", xd_rot, ", xd_ldp: ", xd_ldp, ", xd_fdp: ", xd_fdp, ", xd_bdp: ", xd_bdp, ", xd_rdp: ", xd_rdp, ", xd_max_val: ", xd_max_val, ", xd_ldpo: ", xd_ldpo, ", xd_fdpo: ", xd_fdpo, ", xd_bdpo: ", xd_bdpo, ", xd_rdpo: ", xd_rdpo)
    return xd_ldpo, xd_fdpo, xd_bdpo, xd_rdpo

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
    print("[zero_claw]: started zeroing end effector")
    end_effector_motor.spin_to_position(10, DEGREES, 50, PERCENT, True)
    while(end_effector_motor.torque(TorqueUnits.NM) < 0.5):
        print("[zero_claw]: while loop - ", "end_effector_motor torque: ", end_effector_motor.torque(TorqueUnits.NM))
        end_effector_motor.spin(FORWARD, 50)
    else:
        end_effector_motor.stop()
        end_effector_motor.reset_position()
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
    
    if ato_single_color == -1:
        ato_objects_fruit = ai_vision_2.take_snapshot(ai_vision.ALL_COLORS)
        print("[allign_to_object]: all colors")
    elif ato_single_color == "ai_vision_2__purple_fruit":
        ato_objects_fruit = ai_vision_2.take_snapshot(ai_vision_2__purple_fruit)
        print("[allign_to_object]: purple")
    elif ato_single_color == "ai_vision_2__green_fruit":
        ato_objects_fruit = ai_vision_2.take_snapshot(ai_vision_2__green_fruit)
        print("[allign_to_object]: green")
    elif ato_single_color == "ai_vision_2__orange_fruit":
        ato_objects_fruit = ai_vision_2.take_snapshot(ai_vision_2__orange_fruit)
        print("[allign_to_object]: orange")
    else:
        print("[allign_to_object]: bad ato input", "ato_single_color: ", ato_single_color)

    ato_strafe_Y = 0
    ato_rot = 0
    ato_complete = False
    ato_target_fruit = None
    ato_target_fruit_min_width = 20
    ato_found_fruit = False

    ato_target_fruit = ato_objects_fruit[0]
    
    if (ato_target_fruit is not None) and (ato_target_fruit.width >= ato_target_fruit_min_width):
        ato_found_fruit = True
        print("[allign_to_object]: ato_found_fruit? ", ato_found_fruit, ", number_of_objects: ", len(ato_objects_fruit))
        ato_error_rot = ato_target_fruit.centerX - CENTER_X
        ato_rot = ato_error_rot * 0.15

        if ((not(ato_error_rot > 3)) and (not(ato_error_rot < -3))):
            ato_rot = 0

        ato_TARGET_DISTANCE = 127
        ato_error_Y = ato_TARGET_DISTANCE - ato_target_fruit.width
        ato_strafe_Y = ato_error_Y * 0.2

        if ((not(ato_strafe_Y > 3)) and (not(ato_strafe_Y < -3))):
            ato_strafe_Y = 0

        if((abs(ato_strafe_Y) < 3) and (abs(ato_rot) < 3)):
            ato_complete = True
    else:
        ato_found_fruit = False
        print("[allign_to_object]: no fruit found")
    print("[allign_to_object]: ", "ato_strafe_X: ", 0, ", ato_strafe_Y: ", ato_strafe_Y, ", ato_rot: ", ato_rot, ", ato_complete? ", ato_complete, ", ato_found_fruit? ", ato_found_fruit)
    return(0, ato_strafe_Y, ato_rot, ato_complete, ato_found_fruit)

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

def rotate(r_angle):
    print("[rotate]: ", "r_angle: ", r_angle)

def drive():
    print("[drive]: ")

def climbRamp():
    print("[climbRamp]: ")

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
                    gtb_atat_strafe_X = 20 # ----
                if (gtb_atat_complete2 == True) and (gtb_atat_found_tag2 == True):
                    gtb_s1 = False
            if gtb_gtb_locked_tag == 21:
                gtb_atat_strafe_X, gtb_atat_strafe_Y, gtb_atat_rot, gtb_atat_complete3, gtb_atat_found_tag3, gtb_locked_tag3 = allign_to_apriltag(gtb_bin_number, True)
                if gtb_atat_found_tag3 == False:
                    if gtb_bin_number == 20:
                        gtb_atat_strafe_X = -20 # ----
                    else:
                        gtb_atat_strafe_X = 20 # ----
                if (gtb_atat_complete3 == True) and (gtb_atat_found_tag3 == True):
                    gtb_s1 = False
            if gtb_gtb_locked_tag == 22:
                gtb_atat_strafe_X, gtb_atat_strafe_Y, gtb_atat_rot, gtb_atat_complete4, gtb_atat_found_tag4, gtb_locked_tag4 = allign_to_apriltag(gtb_bin_number, True)
                if gtb_atat_found_tag4 == False:
                    if gtb_bin_number == 23:
                        gtb_atat_strafe_X = 20 # ----
                    else:
                        gtb_atat_strafe_X = -20 # ----
                if (gtb_atat_complete4 == True) and (gtb_atat_found_tag4 == True):
                    gtb_s1 = False
            if gtb_gtb_locked_tag == 23:
                gtb_atat_strafe_X, gtb_atat_strafe_Y, gtb_atat_rot, gtb_atat_complete5, gtb_atat_found_tag5, gtb_locked_tag5 = allign_to_apriltag(gtb_bin_number, True)
                if gtb_atat_found_tag5 == False:
                    gtb_atat_strafe_X = -20 # ----
                if (gtb_atat_complete5 == True) and (gtb_atat_found_tag5 == True):
                    gtb_s1 = False
    if gtb_atat_complete0 == True:
        gtb_complete = True
    print("[go_to_bin]: ", "gtb_s1? ", gtb_s1, ", gtb_atat_found_tag0? ", gtb_atat_found_tag0, ", gtb_gtb_locked_tag: ", gtb_gtb_locked_tag)
    return (gtb_atat_strafe_X, gtb_atat_strafe_Y, gtb_atat_rot, gtb_complete)

def auto_cycle():
    print()

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
        else:
            right_elevator_motor.stop()
            left_elevator_motor.stop()

        if controller.buttonL2.pressing():
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
                controller.rumble('.')

        # Fruit ---------------
        m_ato_strafe_X = 0
        m_ato_strafe_Y = 0
        m_ato_rot = 0

        if (controller.buttonB.pressing()):
            m_ato_strafe_X, m_ato_strafe_Y, m_ato_rot, m_ato_complete, m_ato_found_fruit = allign_to_object("ai_vision_2__purple_fruit")
            if m_ato_found_fruit == False:
                controller.rumble('-')
            if (m_ato_complete == True) and (m_ato_found_fruit ==True):
                end_effector_motor.spin(REVERSE, 100)


        # Drive logic ---------
        m_c_X = controller.axis4.position()
        m_c_Y = controller.axis3.position()
        m_c_rot = controller.axis1.position()

        current_heading = inertial.heading()

        m_c_fo_X, m_c_fo_Y = field_oriented(m_c_X, m_c_Y, current_heading)
        m_atat_ro_X, m_atat_ro_Y = field_oriented(m_atat_strafe_X, m_atat_strafe_Y, 0)
        m_ato_ro_X, m_ato_ro_Y = field_oriented(m_ato_strafe_X, m_ato_strafe_Y, 0)

        m_x = m_c_fo_X + m_atat_ro_X + m_ato_ro_X
        m_y = m_c_fo_Y + m_atat_ro_Y + m_ato_ro_Y
        m_rot = m_atat_rot + m_ato_rot + m_c_rot

        m_ldp, m_fdp, m_bdp, m_rdp = x_drive(m_x, m_y, m_rot)
        set_drive(m_ldp, m_fdp, m_bdp, m_rdp)

    # Auto mode = runs a program and stops after some time ----
    elif state == AUTO:
        # if auto_state == RAMP_START:
        #     print("auto_state: ", auto_state)
        #     #  Rotate to 45 deg. Move onto ramp
        #     # When on ramp (using "get_tilt") switch to RAMP_MID

        # elif auto_state == RAMP_MID:
        #     print("auto_state: ", auto_state)
        #     # Locked rotation at 45 deg. Move up the ramp
        #     # When finished climbing ramp (using "get_tilt") switch to RAMP_TOP

        # elif auto_state == RAMP_TOP:
            
        #     print("auto_state: ", auto_state)
        #     # Switches between finding fruit and finding bins based on if it has a fruit in the claw

        a_gtb_atat_strafe_X = 0
        a_gtb_atat_strafe_Y = 0
        a_gtb_atat_rot = 0
        a_gtb_complete = False
        
        # if controller.buttonY.pressing():
        #     if a_bin <= 20:
        #         a_bin = 20
        #     if a_bin > 20:
        #         a_bin = a_bin - 1

        # if controller.buttonA.pressing():
        #     if a_bin >= 23:
        #         a_bin = 23
        #     if a_bin < 23:
        #         a_bin = a_bin + 1

        if controller.buttonL1.pressing():
            a_bin = 20
        if controller.buttonL2.pressing():
            a_bin = 21
        if controller.buttonR2.pressing():
            a_bin = 22
        if controller.buttonR1.pressing():
            a_bin = 23

        if controller.buttonX.pressing():
            a_gtb_atat_strafe_X, a_gtb_atat_strafe_Y, a_gtb_atat_rot, a_gtb_complete = go_to_bin(a_bin)

        if controller.buttonX.pressing() == False:
            gtb_s1 = False
            gtb_gtb_locked_tag = -1

        a_c_X = controller.axis4.position()
        a_c_Y = controller.axis3.position()
        a_c_rot = controller.axis1.position()

        current_heading = inertial.heading()

        a_c_fo_X, a_c_fo_Y = field_oriented(a_c_X, a_c_Y, current_heading)
        a_gtb_ro_X, a_gtb_ro_Y = field_oriented(a_gtb_atat_strafe_X, a_gtb_atat_strafe_Y, 0)

        m_x = a_gtb_ro_X + a_c_fo_X
        m_y = a_gtb_ro_Y + a_c_fo_Y
        m_rot = a_gtb_atat_rot + m_c_rot

        m_ldp, m_fdp, m_bdp, m_rdp = x_drive(m_x, m_y, m_rot)
        set_drive(m_ldp, m_fdp, m_bdp, m_rdp)

        # print("[a]: ", "a_bin: ", a_bin)

        timer.event(auto_complete, auto_time * 1000)
        
    # Update display ------------------------------------------
    # Displays info on the brain screen
    display_info()

    # Program speed limit -------------------------------------
    # Waits for 20 milliseconds [50 times per second (50 Hz)] to prevent CPU overload and flickering, jittering or behaving inconsistently
    wait(80, MSEC)
