from vex import *
import math

brain       = Brain()
controller  = Controller()
front_right = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
front_left  = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
back_right  = Motor(Ports.PORT14, GearSetting.RATIO_18_1, True)
back_left   = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
IMU         = Inertial(Ports.PORT15)

# --- PID gains ---
kP             = 0.65
kI             = 0.002
kD             = 0.08
DEADBAND       = 2.0
INTEGRAL_LIMIT = 30.0
SPEED_LIMIT    = 0.5

# --- PID state ---
prev_error = 0.0
integral   = 0.0
target     = 0.0

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
#  Non-blocking drive/strafe  (call inside your own loop)
#  PID heading correction is applied on every call.
# ─────────────────────────────────────────────

def drive_forward(speed=100):
    turn = compute_pid_turn()
    x, y = field_oriented(0, speed, IMU.heading(DEGREES))
    fl, fr, bl, br = x_drive_normalized(x, y, turn)
    set_motors(fl, fr, bl, br)

def drive_backward(speed=100):
    turn = compute_pid_turn()
    x, y = field_oriented(0, -speed, IMU.heading(DEGREES))
    fl, fr, bl, br = x_drive_normalized(x, y, turn)
    set_motors(fl, fr, bl, br)

def strafe_left(speed=100):
    turn = compute_pid_turn()
    x, y = field_oriented(-speed, 0, IMU.heading(DEGREES))
    fl, fr, bl, br = x_drive_normalized(x, y, turn)
    set_motors(fl, fr, bl, br)

def strafe_right(speed=100):
    turn = compute_pid_turn()
    x, y = field_oriented(speed, 0, IMU.heading(DEGREES))
    fl, fr, bl, br = x_drive_normalized(x, y, turn)
    set_motors(fl, fr, bl, br)

# ─────────────────────────────────────────────
#  Blocking turn commands  (settle before returning)
# ─────────────────────────────────────────────

def turn_to(angle_deg, timeout_ms=3000):
    """Turn to an absolute field heading and lock it as the new target."""
    global target, prev_error, integral
    target     = angle_deg % 360
    integral   = 0.0
    prev_error = 0.0
    elapsed    = 0
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

def turn_left(degrees, timeout_ms=3000):
    """Turn left (CCW) by a relative number of degrees."""
    turn_to((target - degrees) % 360, timeout_ms)

def turn_right(degrees, timeout_ms=3000):
    """Turn right (CW) by a relative number of degrees."""
    turn_to((target + degrees) % 360, timeout_ms)

# ─────────────────────────────────────────────
#  Autonomous routine  ← edit this
# ─────────────────────────────────────────────

def run_auton():

    print("hello")

# ─────────────────────────────────────────────
#  Start on button press (Button A)
# ─────────────────────────────────────────────


while True:
    if controller.buttonA.pressing():
        brain.screen.clear_screen()
        brain.screen.print("Running auton...")
        run_auton()
        brain.screen.clear_screen()
        brain.screen.print("Auton complete.")
        break
    wait(20, MSEC)
