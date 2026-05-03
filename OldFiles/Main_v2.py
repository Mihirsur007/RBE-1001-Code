# -----------------------------
# Imports
# -----------------------------
from vex import *
import math
import time

# --------
# Timer
# --------
timer = Timer()

# -----------------------------
# BRAIN & CONTROLLER
# -----------------------------
brain=Brain()
controller = Controller()

# -----------------------------
# MOTORS (X-drive)
# -----------------------------
# From perspective of siting in the robot
# Front Left  = 028 (11)
# Front Right = 029 (12)
# Back Left   = 026 (13)
# Back Right  = 025 (14)
#                         Motor(port,         gears,                  reverse)
front_left_drive_motor  = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
front_right_drive_motor = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
back_left_drive_motor   = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
back_right_drive_motor  = Motor(Ports.PORT14, GearSetting.RATIO_18_1, True)

# -----------------------------
# INERTIAL SENSOR
# -----------------------------
# Center = 004 (15)
inertial = Inertial(Ports.PORT15)

# -----------------------------
# STATES, VARS, and CONSTANTS
# -----------------------------
STANDBY       = 0
MOVE          = 1
CALIBRATION   = 2
AUTO          = 3

state = STANDBY
prev_state = None
target_heading = 0
wheel_diameter = 4 # in
wheel_circumference = math.pi*wheel_diameter
drivetrain_diameter = 11.75

do_auto = True
auto_time = 1 # seconds - 0 for no auto

# rotation pid values
Kp = 0.8
Ki = 0.002
Kd = 0.8
heading_integral = 0
last_error = 0
last_time = time.time()

pos_x = 0.0
pos_y = 0.0

vel_x = 0.0
vel_y = 0.0

target_distance = 24  # inches
distance_error = 0

# -----------------------------
# HELPER FUNCTIONS
# -----------------------------
# Stops all drive motors
def stop_all():
    front_left_drive_motor.stop()
    front_right_drive_motor.stop()
    back_left_drive_motor.stop()
    back_right_drive_motor.stop()

# Sends all drive motor commands together
def set_drive(fl, fr, bl, br):
    front_left_drive_motor.spin(FORWARD, fl, PERCENT)
    front_right_drive_motor.spin(FORWARD, fr, PERCENT)
    back_left_drive_motor.spin(FORWARD, bl, PERCENT)
    back_right_drive_motor.spin(FORWARD, br, PERCENT)

# Makes x and y field-oriented instead of robot-oriented using the current heading
def field_oriented(x, y, heading):
    rad = math.radians(heading)
    temp = y * math.cos(rad) + x * math.sin(rad)
    x = -y * math.sin(rad) + x * math.cos(rad)
    y = temp
    return x, y

# Combines strafe (left/right), forward/backward, and rotation (spin) into 4 motor speeds for each wheel.
# Then normalizes values to prevent a motor going >100% speed
def x_drive(x, y, rot):
    fl = y + x + rot
    fr = y - x - rot
    bl = y - x + rot
    br = y + x - rot
    # Normalize
    max_val = max(abs(fl), abs(fr), abs(bl), abs(br), 100)
    return fl / max_val * 100, fr / max_val * 100, bl / max_val * 100, br / max_val * 100

# When run displays info on the brain screen
def display_info():
    # Clear the screen (set it solid black)
    brain.screen.set_fill_color(Color.BLACK)
    brain.screen.draw_rectangle(10, 0, 470, 240)

    # Draw colored state bar (10px wide on left)
    color_map = {STANDBY: Color.GREEN, MOVE: Color.PURPLE, AUTO: Color.BLUE, CALIBRATION: Color.RED}
    brain.screen.set_fill_color(color_map[state])
    brain.screen.draw_rectangle(0, 0, 30, 240)

    mode_text_map = {STANDBY: "STANDBY", MOVE: "DRIVE+CORRECT", CALIBRATION: "CALIBRATION", AUTO: "AUTO"}
    brain.screen.set_cursor(1, 1)
    brain.screen.set_fill_color(color_map[state])
    brain.screen.print(mode_text_map[state])

    brain.screen.set_cursor(5, 5)
    brain.screen.print('Pos x,y: ', pos_x, pos_y, 'Vel x,y: ', vel_x, vel_y)

# runs after the auto time is up
def auto_complete():
    global state
    state = MOVE

# uses inertial sensor aceleration to keep track of position. DOES NOT WORK
def update_position():
    global pos_x, pos_y, vel_x, vel_y, last_time

    # --- TIME ---
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    if dt <= 0:
        return

    # --- RAW ACCEL (robot frame, in g) ---
    ax = inertial.acceleration(AxisType.XAXIS)
    ay = inertial.acceleration(AxisType.YAXIS)

    # Convert g → m/s^2
    ax *= 9.81
    ay *= 9.81

    # --- DEADZONE (reduce noise) ---
    if abs(ax) < 0.05: ax = 0
    if abs(ay) < 0.05: ay = 0

    # --- HEADING (degrees → radians) ---
    heading_deg = inertial.heading()
    heading = math.radians(heading_deg)

    # --- ROTATE INTO FIELD FRAME ---
    field_ax = ax * math.cos(heading) - ay * math.sin(heading)
    field_ay = ax * math.sin(heading) + ay * math.cos(heading)

    # --- INTEGRATE ACCEL → VELOCITY ---
    vel_x += field_ax * dt
    vel_y += field_ay * dt

    # --- DAMPING (reduces drift) ---
    vel_x *= 0.98
    vel_y *= 0.98

    # --- INTEGRATE VELOCITY → POSITION ---
    pos_x += vel_x * dt
    pos_y += vel_y * dt

# --------------------------------------------------------------------------------------------------------------------------------------------------------
# MAIN LOOP
# --------------------------------------------------------------------------------------------------------------------------------------------------------
# Runs/loops forever
while True:

    # --- STATE TRANSITIONS ---
    # Changes modes based on butons pressed.
    # Sets target heading when changing to move or correction mode
    
    # X button = calabration mode
    # Up arrow = move mode
    # Down arrow = correction mode
    # Any bumper = standby mode
    if controller.buttonX.pressing():
        state = CALIBRATION

    elif controller.buttonUp.pressing():
        if state != MOVE:
            target_heading = inertial.heading()
        state = MOVE

    elif controller.buttonDown.pressing():
        if state != AUTO:
            target_heading = inertial.heading()
            start_position = (front_left_drive_motor.position(DEGREES) + front_right_drive_motor.position(DEGREES) + 
                  back_left_drive_motor.position(DEGREES) + back_right_drive_motor.position(DEGREES)) / 4
        state = AUTO

    elif (controller.buttonL1.pressing() or controller.buttonL2.pressing() or
          controller.buttonR1.pressing() or controller.buttonR2.pressing()):
        state = STANDBY

    if controller.buttonY.pressing():
            target_distance = 24

    # --- STATE LOGIC ---
    # Standby mode = stops all drive motors
    # Calabration mode = stops all drive motors and calabrates inertial sensor - ALL IMPUTS ARE BLOCKED DURING CALIBRATION!!!
    # Move mode = gets joystick pos and moves drive motors
    # Correction mode = tries it's best to remain facing the direction it was facing when this mode was acctivated
    if state == STANDBY:
        stop_all()

    elif state == CALIBRATION:
        stop_all()
        brain.screen.set_fill_color(Color.RED)
        brain.screen.draw_rectangle(30, 0, 30, 240)
        inertial.calibrate()
        while inertial.is_calibrating():
            wait(50, MSEC)
        brain.screen.set_fill_color(Color.BLACK)
        brain.screen.draw_rectangle(30, 0, 30, 240)
        state = STANDBY

    elif state == MOVE:
        # # Read controller
        # x = controller.axis4.position()
        # y = controller.axis3.position()
        # rot = controller.axis1.position()

        # current_heading = inertial.heading()

        # # --- HEADING HOLD LOGIC ---
        # # If driver is NOT rotating (deadband)
        # if abs(rot) < 5:
        #     error = target_heading - current_heading

        #     # Wrap error to [-180, 180]
        #     if error > 180:
        #         error -= 360
        #     elif error < -180:
        #         error += 360

        #     correction = error * 0.5
        # else:
        #     # Driver is rotating → update target heading
        #     target_heading = current_heading
        #     correction = 0

        # # Field-oriented drive
        # x, y = field_oriented(x, y, current_heading)

        # # Combine driver rotation + correction
        # fl, fr, bl, br = x_drive(x, y, rot + correction)

        # set_drive(fl, fr, bl, br)

    
        x = controller.axis4.position()
        y = controller.axis3.position()
        rot = controller.axis1.position()

        current_heading = inertial.heading()

        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        if abs(rot) < 5:
            error = target_heading - current_heading

            if error > 180:
                error -= 360
            elif error < -180:
                error += 360

            heading_integral += error * dt
            derivative = (error - last_error) / dt if dt > 0 else 0

            correction = Kp*error + Ki*heading_integral + Kd*derivative
            last_error = error

        else:
            target_heading = current_heading
            heading_integral = 0
            last_error = 0
            correction = 0

        x, y = field_oriented(x, y, current_heading)

        fl, fr, bl, br = x_drive(x, y, rot + correction)
        set_drive(fl, fr, bl, br)

    elif state == AUTO:

        # takes the current heading and 
        current_heading = inertial.heading()
        error = target_heading - current_heading

        # Wrap error to [-180, 180]
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        
        # rotation pid
        kp_turn = 0.8
        correction = error * kp_turn

        
        # distance pid
        current_position = (front_left_drive_motor.position(DEGREES) + front_right_drive_motor.position(DEGREES) + 
                        back_left_drive_motor.position(DEGREES) + back_right_drive_motor.position(DEGREES)) / 4
        distance_traveled = ((current_position - start_position) / 360) * wheel_circumference

        print(distance_error, target_distance, distance_traveled)
        if (distance_traveled > 23.5):
            target_distance = -0

        distance_error = target_distance - distance_traveled
        kp_pos = 1.5
        y_output = distance_error * kp_pos
        y_output = max(min(y_output, 100), -100)

        '''
        target_x = 0
        target_y = 24

        # Current estimated position
        #current_x = distance_traveled * math.cos(math.radians(current_heading))
        current_y = distance_traveled * math.sin(math.radians(current_heading))

        # Position error
        #error_x = target_x - current_x
        error_y = target_y - current_y

        # Proportional outputs
        kp_pos = 0.8
        #x_output = error_x * kp_pos
        y_output = error_y * kp_pos

        # Heading correction (if you want it separate)
        heading_error = target_heading - current_heading
        correction = heading_error * kp_turn
        '''
        # Drive motors
        fl, fr, bl, br = x_drive(0, y_output, correction)
        set_drive(fl, fr, bl, br)
        
        display_info()
        #timer.clear()
        #timer.event(auto_complete, auto_time*1000)
        
    # --- DISPLAY ---
    # Displays info on the brain screen
    display_info()

    # Waits for 20 milliseconds [50 times per second (50 Hz)] to prevent CPU overload and flickering, jittering or behaving inconsistently
    wait(20, MSEC)
