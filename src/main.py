from vex import *

# --- devices -----------------------------------------------------------------
brain = Brain()
controller = Controller()

# Drivetrain Motor Defs.
left_front = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
left_back = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
right_front = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)
right_back = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)

# Grouping the motors
left_drive = MotorGroup(left_front, left_back)
right_drive = MotorGroup(right_front, right_back)

drivetrain = DriveTrain(left_drive, right_drive, 319.19, 320, 250, MM, 1)

arm_left = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
arm_right = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)
arm = MotorGroup(arm_left, arm_right)

claw = Motor(Ports.PORT4, GearSetting.RATIO_6_1, False)
pivot = Motor(Ports.PORT5, GearSetting.RATIO_18_1, True)

# Vision sensor signatures
vision_20__YELLO  = Signature(1, 1873, 2981, 2427, -4389, -3621, -4005, 3.0, 0)
vision_20__BLU    = Signature(2, -3211, -1985, -2598, 1201, 2450, 1825, 3.0, 0)
vision_20__ORANGG = Signature(3, 5420, 7610, 6515, -2200, -1150, -1675, 3.0, 0)

vision_20 = Vision(Ports.PORT7, vision_20__YELLO, vision_20__BLU, vision_20__ORANGG)

# --- constants ---------------------------------------------------------------
ZERO_CURRENT_AMP = 5.0
ZERO_STARTUP_WAIT = 0.3
ZERO_TIMEOUT_SEC = 3.0
MATCH_LOAD_DEG = 30
FIRST_STACK_DEG = 120
PLACE_VOLTS = 8
PLACE_WAIT_SEC = 0.2
DRIVE_DEADBAND = 8
COAST_BELOW_DEG = 30
CLAW_OPEN_DEG = 90
CLAW_CLOSED_DEG = 0
CLAW_VELOCITY = 100

arm_using_preset = False

# --- claw & arm functions ---------------------------------------------------
def close_claw():
    claw.set_stopping(HOLD)
    claw.spin(FORWARD, CLAW_VELOCITY, PERCENT)

def open_claw():
    claw.set_stopping(HOLD)
    claw.spin(REVERSE, CLAW_VELOCITY, PERCENT)

def claw_control():
    if controller.buttonL1.pressing():
        close_claw()
    elif controller.buttonL2.pressing():
        open_claw()
    else:
        claw.stop()

def arm_spin_volts(direction, volts):
    arm_left.spin(direction, volts, VOLT)
    arm_right.spin(direction, volts, VOLT)

def zero_arm():
    global arm_using_preset
    arm_using_preset = False
    arm.spin(REVERSE)
    wait(ZERO_STARTUP_WAIT, SECONDS)
    elapsed = 0.0
    while arm.current(CurrentUnits.AMP) <= ZERO_CURRENT_AMP and elapsed < ZERO_TIMEOUT_SEC:
        wait(20, MSEC)
        elapsed += 0.02
    arm.set_position(0, DEGREES)
    controller.rumble("-")

def go_match_load():
    global arm_using_preset
    arm_using_preset = True
    arm.set_stopping(HOLD)
    arm.spin_to_position(MATCH_LOAD_DEG, DEGREES, wait=False)

def go_first_stack():
    global arm_using_preset
    arm_using_preset = True
    arm.set_stopping(HOLD)
    arm.spin_to_position(FIRST_STACK_DEG, DEGREES, wait=False)

def place_sequence():
    global arm_using_preset
    arm_using_preset = True
    arm_spin_volts(REVERSE, PLACE_VOLTS)
    wait(PLACE_WAIT_SEC, SECONDS)
    open_claw()
    arm.stop()
    arm_using_preset = False

def lift_and_drop():
    """Automated macro to raise arm and release claw"""
    global arm_using_preset
    arm_using_preset = True
    arm.set_stopping(HOLD)
    arm.spin_to_position(FIRST_STACK_DEG, DEGREES, wait=True)
    open_claw()
    wait(0.4, SECONDS)
    claw.stop()
    arm_using_preset = False

def apply_arm_hold_or_coast():
    if arm.position(DEGREES) < COAST_BELOW_DEG:
        arm.set_stopping(COAST)
    else:
        arm.set_stopping(HOLD)

# --- drive & manual control --------------------------------------------------
def arcade_drive():
    raw_forward = controller.axis3.position()
    raw_turn = controller.axis1.position()

    if abs(raw_forward) < DRIVE_DEADBAND and abs(raw_turn) < DRIVE_DEADBAND:
        drivetrain.stop()
        return

    # Exponential scaling for smoother acceleration
    forward = (raw_forward ** 3) / 10000.0
    turn = (raw_turn ** 3) / 10000.0

    left = forward + turn
    right = forward - turn

    biggest = max(abs(left), abs(right), 100.0)
    left = (left / biggest) * 100.0
    right = (right / biggest) * 100.0

    left_drive.spin(FORWARD, left, PERCENT)
    right_drive.spin(FORWARD, right, PERCENT)

def run_arm_manual():
    global arm_using_preset
    up = controller.buttonR1.pressing()
    down = controller.buttonR2.pressing()

    if up and not down:
        arm_using_preset = False
        apply_arm_hold_or_coast()
        arm.spin(FORWARD, 100, PERCENT)
    elif down and not up:
        arm_using_preset = False
        apply_arm_hold_or_coast()
        arm.spin(REVERSE, 100, PERCENT)
    elif not arm_using_preset:
        apply_arm_hold_or_coast()
        arm.stop()

def pivot_up():
    pivot.set_stopping(HOLD)
    pivot.spin(REVERSE, CLAW_VELOCITY, PERCENT)

def pivot_down():
    pivot.set_stopping(HOLD)
    pivot.spin(FORWARD, CLAW_VELOCITY, PERCENT)

def pivot_control():
    if controller.buttonUp.pressing():
        pivot_up()
    elif controller.buttonDown.pressing():
        pivot_down()
    else:
        pivot.stop()

# --- autonomous & vision ----------------------------------------------------
def block_turn():
    claw.spin_for(FORWARD, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 200, MM)
    claw.spin_for(REVERSE, 90, DEGREES)
    arm.spin_to_position(180, DEGREES)
    drivetrain.turn_for(RIGHT, 90, DEGREES)

def process_vision():
    vision_20.take_snapshot(vision_20__YELLO)
    if vision_20.object_count() > 0:
        drivetrain.turn_for(RIGHT, 90, DEGREES)
    else:
        vision_20.take_snapshot(vision_20__BLU)
        if vision_20.object_count() > 0:
            drivetrain.turn_for(LEFT, 90, DEGREES)
        else:
            vision_20.take_snapshot(vision_20__ORANGG)
            if vision_20.object_count() > 0:
                if vision_20.largest_object().width > 100:
                    drivetrain.drive_for(REVERSE, 200, MM)
                else:
                    block_turn()

def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("Running Autonomous...")
    drivetrain.set_drive_velocity(60, PERCENT)
    drivetrain.set_turn_velocity(40, PERCENT)
    arm.set_stopping(HOLD)
    claw.set_stopping(HOLD)
    
    close_claw()
    wait(0.3, SECONDS)
    arm.spin_to_position(MATCH_LOAD_DEG, DEGREES, wait=True)
    drivetrain.drive_for(FORWARD, 600, MM)
    
    vision_20.take_snapshot(vision_20__ORANGG)
    if vision_20.object_count() > 0:
        center_x = vision_20.largest_object().centerX
        if center_x < 140:
            drivetrain.turn_for(LEFT, 10, DEGREES)
        elif center_x > 180:
            drivetrain.turn_for(RIGHT, 10, DEGREES)
        drivetrain.drive_for(FORWARD, 300, MM)
    else:
        drivetrain.drive_for(FORWARD, 300, MM)
        
    lift_and_drop()
    drivetrain.drive_for(REVERSE, 400, MM)
    zero_arm()

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    drivetrain.set_stopping(COAST)
    zero_arm()
    while True:
        arcade_drive()
        run_arm_manual()
        claw_control()
        pivot_control()
        wait(20, MSEC)

# Button Binds
controller.buttonY.pressed(go_match_load)
controller.buttonA.pressed(place_sequence)
controller.buttonB.pressed(go_first_stack)
controller.buttonX.pressed(zero_arm)
controller.buttonRight.pressed(lift_and_drop)

# Startup Initialization
arm.set_velocity(100, PERCENT)
arm.set_stopping(HOLD)
claw.set_velocity(CLAW_VELOCITY, PERCENT)
claw.set_stopping(HOLD)
claw.set_position(CLAW_OPEN_DEG, DEGREES)

comp = Competition(user_control, autonomous)