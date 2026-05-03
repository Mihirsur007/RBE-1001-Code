from vex import *
import math

brain       = Brain()
controller  = Controller()
front_right = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
front_left  = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
back_right  = Motor(Ports.PORT14, GearSetting.RATIO_18_1, True)
back_left   = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
IMU         = Inertial(Ports.PORT15)
USS         = Sonar(brain.three_wire_port.a)
USSL    = Sonar(brain.three_wire_port.e)

# --- PID gains ---
kP             = 0.70
kI             = 0.005
kD             = 0.1
DEADBAND       = 2.0
INTEGRAL_LIMIT = 30.0
SPEED_LIMIT    = 0.5

# --- PID state ---
prev_error = 0.0
integral   = 0.0
target     = 0.0

# --- Auton state ---
IDLE          = 0
RUNNING       = 1
current_state = IDLE

# --- Heading hold state ---
heading_hold_active = True  # Always ON unless explicitly suppressed (e.g. during turn_to)

# --- Calibrate IMU ---
IMU.calibrate()
while IMU.is_calibrating():
    wait(50, MSEC)
target = IMU.heading(DEGREES)

# ─────────────────────────────────────────────
#  Core helpers
# ─────────────────────────────────────────────

def field_oriented(x, y, heading_deg):
    rad = math.radians(-heading_deg)
    fx  = x * math.cos(rad) - y * math.sin(rad)
    fy  = x * math.sin(rad) + y * math.cos(rad)
    return fx, fy

def x_drive_normalized(x, y, rot):
    fl = y + x + rot
    fr = y - x - rot
    bl = y - x + rot
    br = y + x - rot
    max_val = max(abs(fl), abs(fr), abs(bl), abs(br), 100)
    scale   = 100.0 / max_val * SPEED_LIMIT
    return fl * scale, fr * scale, bl * scale, br * scale

def compute_pid_turn():
    global prev_error, integral
    error = target - IMU.heading(DEGREES)
    if error >  180: error -= 360
    if error < -180: error += 360

    if abs(error) > DEADBAND:
        integral  += error
        integral   = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, integral))
        derivative = error - prev_error
        turn       = kP * error + kI * integral + kD * derivative
    else:
        integral = 0.0
        turn     = 0.0

    prev_error = error
    return turn

def set_motors(fl, fr, bl, br):
    front_left .spin(FORWARD, fl, PERCENT)
    front_right.spin(FORWARD, fr, PERCENT)
    back_left  .spin(FORWARD, bl, PERCENT)
    back_right .spin(FORWARD, br, PERCENT)

def stop_motors():
    front_left .stop()
    front_right.stop()
    back_left  .stop()
    back_right .stop()

# ─────────────────────────────────────────────
#  Background heading-hold thread
#  Runs forever. When no translation is commanded,
#  it fires correction-only motor outputs so the
#  robot always fights drift back to `target`.
# ─────────────────────────────────────────────

# Shared translation intent set by drive helpers each tick
_tx = 0.0   # field-frame X (strafe)
_ty = 0.0   # field-frame Y (forward)

def heading_hold_thread():
    """Continuously applies PID heading correction combined with
    whatever translation was requested this tick."""
    global _tx, _ty
    while True:
        if heading_hold_active:
            turn = compute_pid_turn()
            # Use the most-recently requested translation vector
            fl, fr, bl, br = x_drive_normalized(_tx, _ty, turn)
            set_motors(fl, fr, bl, br)
            # Clear translation intent — drive helpers repopulate each tick
            _tx = 0.0
            _ty = 0.0
        wait(20, MSEC)

# Kick off the background thread immediately
Thread(heading_hold_thread)

# ─────────────────────────────────────────────
#  Non-blocking drive/strafe
#  Now just post the desired translation; the
#  heading-hold thread merges it with PID turn.
# ─────────────────────────────────────────────

def drive_forward(speed=50):
    global _tx, _ty
    fx, fy = field_oriented(0, speed, IMU.heading(DEGREES))
    _tx, _ty = fx, fy

def drive_backward(speed=50):
    global _tx, _ty
    fx, fy = field_oriented(0, -speed, IMU.heading(DEGREES))
    _tx, _ty = fx, fy

def strafe_left(speed=50):
    global _tx, _ty
    fx, fy = field_oriented(-speed, 0, IMU.heading(DEGREES))
    _tx, _ty = fx, fy

def strafe_right(speed=50):
    global _tx, _ty
    fx, fy = field_oriented(speed, 0, IMU.heading(DEGREES))
    _tx, _ty = fx, fy

# ─────────────────────────────────────────────
#  Blocking turn commands
# ─────────────────────────────────────────────

def turn_to(angle_deg, timeout_ms=3000):
    """Turn to an absolute field heading and lock it as the new target."""
    global target, prev_error, integral, heading_hold_active
    target              = angle_deg % 360
    integral            = 0.0
    prev_error          = 0.0
    heading_hold_active = False   # take manual control of motors
    elapsed             = 0
    while elapsed < timeout_ms:
        turn = compute_pid_turn()
        fl, fr, bl, br = x_drive_normalized(0, 0, turn)
        set_motors(fl, fr, bl, br)
        wait(20, MSEC)
        elapsed += 20
        error = target - IMU.heading(DEGREES)
        if error >  180: error -= 360
        if error < -180: error += 360
        if abs(error) <= DEADBAND:
            break
    stop_motors()
    integral            = 0.0
    prev_error          = 0.0
    heading_hold_active = True    # hand back to background thread

def turn_left(degrees, timeout_ms=3000):
    turn_to((target - degrees) % 360, timeout_ms)

def turn_right(degrees, timeout_ms=3000):
    turn_to((target + degrees) % 360, timeout_ms)

# ─────────────────────────────────────────────
#  Autonomous routine
# ─────────────────────────────────────────────

def run_auton():
    LEFT_TARGET = 200  # desired distance from left wall in MM
    TOLERANCE = 20

    def adjust_lateral():
        left_dist = USSL.distance(MM)
        if left_dist < LEFT_TARGET - TOLERANCE:
            strafe_right()
        elif left_dist > LEFT_TARGET + TOLERANCE:
            strafe_left()

    for i in range(4):
        # Drive to far wall
        while USS.distance(MM) > 175:
            adjust_lateral()
            drive_forward()
        stop_motors()

        # Turn 180 and drive back to base
        turn_right(180)
        while USS.distance(MM) > 175:
            adjust_lateral()
            drive_backward()
        stop_motors()
        
  
# ─────────────────────────────────────────────
#  Button A handler
# ─────────────────────────────────────────────

def handleButtonA():
    global current_state
    if current_state == IDLE:
        print('IDLE -> RUNNING')
        current_state = RUNNING
        run_auton()
    else:
        print(' -> IDLE (E-stop)')
        current_state = IDLE
        stop_motors()

# ─────────────────────────────────────────────
#  Motion checker / handler
# ─────────────────────────────────────────────

wasMoving = False

def checkMotionComplete():
    global wasMoving
    isMoving  = (front_left.is_spinning()  or
                 front_right.is_spinning() or
                 back_left.is_spinning()   or
                 back_right.is_spinning())
    retVal    = wasMoving and not isMoving
    wasMoving = isMoving
    return retVal

def handleMotionComplete():
    global current_state
    if current_state == RUNNING:
        print('Motion complete -> IDLE')
        current_state = IDLE
    else:
        print('Motors stopped')

# ─────────────────────────────────────────────
#  Register callbacks
# ─────────────────────────────────────────────

controller.buttonA.pressed(handleButtonA)

# ─────────────────────────────────────────────
#  Main loop
# ─────────────────────────────────────────────

while True:
    if checkMotionComplete():
        handleMotionComplete()
    print(USS.distance(MM))
    wait(20, MSEC)
