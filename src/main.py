# ---------------------------------------------------------------------------- #
#   Module:       main.py                                                      #
#   Author:       anixas.                                                      #
#   Created:      9/28/2026, 7:15:25 PM                                        #
#   Description:  DR4B + 4-motor drivetrain (VEXcode Blocks tutorial → Python) #
# ---------------------------------------------------------------------------- #
#
# Port map — change these to match your robot:
#   Drive left  : PORT1 (front), PORT2 (back)
#   Drive right : PORT9 (front), PORT10 (back)  — reversed so FORWARD is forward
#   DR4B arm    : PORT3 and PORT8 (PORT8 reversed so they lift together)
#   Claw        : PORT4  blue 6:1 cartridge, 1:1 with 16-tooth gears
#
# Controller (Blocks hats + our extra binds):
#   Left stick Y  (axis 3)  — drive forward / reverse
#   Left stick X  (axis 1)  — turn
#   R1 — DR4B up
#   R2 — DR4B down
#   Y — match-load preset (spin to 30°, don't wait)
#   A — place sequence (8 V down, wait 0.2 s, open claw, stop)
#   B — first-stack preset (tune FIRST_STACK_DEG)
#   X — re-zero the arm
#   L1 — close claw
#   L2 — open claw
#
from vex import *


# --- devices -----------------------------------------------------------------
brain = Brain()
controller = Controller()


# Drivetrain Motor Defs.
left_front = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
left_back = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
right_front = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)
right_back = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)

# Grouping the motors.
left_drive = MotorGroup(left_front, left_back)
right_drive = MotorGroup(right_front, right_back)


drivetrain = DriveTrain(left_drive, right_drive, 319.19, 320, 250, MM, 1)

arm_left = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
arm_right = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)
arm = MotorGroup(arm_left, arm_right)

claw = Motor(Ports.PORT4, GearSetting.RATIO_6_1, False)
pivot = Motor(Ports.PORT5, GearSetting.RATIO_18_1, True)

# --- constants ---------------------------------------------------------------
ZERO_CURRENT_AMP = 5.0
ZERO_STARTUP_WAIT = 0.3     # Blocks: wait 0.3 seconds
ZERO_TIMEOUT_SEC = 3.0
MATCH_LOAD_DEG = 30
FIRST_STACK_DEG = 120
PLACE_VOLTS = 8             # Blocks: spin Arm reverse at 8 volts
PLACE_WAIT_SEC = 0.2
DRIVE_DEADBAND = 5          # A small range of input values around the center of the joystick that the robot's code intentionally ignores
COAST_BELOW_DEG = 30
CLAW_OPEN_DEG = 90
CLAW_CLOSED_DEG = 0
CLAW_VELOCITY = 100

arm_using_preset = False


# Instead of spinning to set positions, the claw moves until the buttons are no longer being pressed. (Kaviysh)
def close_claw():
    claw.set_stopping(HOLD)
    claw.spin(FORWARD, CLAW_VELOCITY, PERCENT)  # Moves Up (Counter-Clockwise)


def open_claw():
    claw.set_stopping(HOLD)
    claw.spin(REVERSE, CLAW_VELOCITY, PERCENT)  # Moves Down (Clockwise)


# Control function to keep the claw from spinning forever. (Kaviysh)    
def claw_control():
    if controller.buttonL1.pressing():
        close_claw()
    elif controller.buttonL2.pressing():
        open_claw()
    else:
        claw.stop()


def arm_spin_volts(direction, volts):
   # MotorGroup.spin only types PERCENT/RPM. Spin each motor so this matches
   # the Blocks "spin Arm reverse at 8 volts" without the type error.
   arm_left.spin(direction, volts, VOLT)
   arm_right.spin(direction, volts, VOLT)


def zero_arm():
   """when driver control → zeroing arm"""
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
   """when button Y pressed → match loading position, don't wait"""
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
   """when button A pressed → place sequence"""
   global arm_using_preset
   arm_using_preset = True
   arm_spin_volts(REVERSE, PLACE_VOLTS)
   wait(PLACE_WAIT_SEC, SECONDS)
   open_claw()
   arm.stop()
   arm_using_preset = False








def apply_arm_hold_or_coast():
   if arm.position(DEGREES) < COAST_BELOW_DEG:
       arm.set_stopping(COAST)
   else:
       arm.set_stopping(HOLD)


def arcade_drive():
   forward = controller.axis3.position()
   turn = controller.axis1.position()
   left = forward + turn
   right = forward - turn

   biggest = max(abs(left), abs(right), 100)
   left = left * 100.0 / biggest
   right = right * 100.0 / biggest

   if abs(left) < DRIVE_DEADBAND and abs(right) < DRIVE_DEADBAND:
       drivetrain.stop()
   else:
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


# Two functions to move the claw's pivot. Set positions are used as manual control is unnecessary and too tedious. (Kaviysh)
def pivot_up():
    pivot.set_stopping(HOLD)
    pivot.spin(REVERSE, CLAW_VELOCITY, PERCENT)  # Moves Up (Counter-Clockwise)


def pivot_down():
    pivot.set_stopping(HOLD)
    pivot.spin(FORWARD, CLAW_VELOCITY, PERCENT)  # Moves Down (Clockwise)




def pivot_control():
    if controller.buttonUp.pressing():
        pivot_up()
    elif controller.buttonDown.pressing():
        pivot_down()
    else:
        pivot.stop()


def autonomous():
   brain.screen.clear_screen()
   brain.screen.print("autonomous")
   drivetrain.set_drive_velocity(50, PERCENT)
   drivetrain.set_turn_velocity(40, PERCENT)


def user_control():
   # when driver control
   brain.screen.clear_screen()
   brain.screen.print("driver control")
   drivetrain.set_stopping(COAST)
   zero_arm()
   while True:
       arcade_drive()
       run_arm_manual()
       # Claw control is deployed here along side the arm's manual code. (Kaviysh)
       claw_control()
       pivot_control()
       wait(20, MSEC)


controller.buttonY.pressed(go_match_load)
controller.buttonA.pressed(place_sequence)
controller.buttonB.pressed(go_first_stack)
controller.buttonX.pressed(zero_arm)

# --- vision sensor setup ---------------------------------------------------
# Assuming Vision20 is connected to PORT7 (update to match your wiring)
vision_20 = Vision(Ports.PORT7)

# Vision signatures defined in VEXcode Vision Utility:
# The utility gives you parameters in this exact format:
# Signature(id, uMin, uMax, uMean, vMin, vMax, vMean, range, type)

vision_20__YELLO = Signature(1, 1873, 2981, 2427, -4389, -3621, -4005, 3.0, 0)
vision_20__BLU   = Signature(2, -3211, -1985, -2598, 1201, 2450, 1825, 3.0, 0)
vision_20__ORANGG = Signature(3, 5420, 7610, 6515, -2200, -1150, -1675, 3.0, 0)
def block_turn():
    """Custom block function defined in Blocks screenshot"""
    claw.spin_for(FORWARD, 90, DEGREES)
    drivetrain.drive_for(FORWARD, 200, MM)
    claw.spin_for(REVERSE, 90, DEGREES)
    arm.spin_to_position(180, DEGREES)
    drivetrain.turn_for(RIGHT, 90, DEGREES)

def process_vision():
    """Vision tracking logic from screenshot"""
    # 1. Check for Yellow object
    vision_20.take_snapshot(vision_20__YELLO)
    if vision_20.object_count > 0:
        drivetrain.turn_for(RIGHT, 90, DEGREES)
    else:
        # 2. Check for Blue object
        vision_20.take_snapshot(vision_20__BLU)
        if vision_20.object_count > 0:
            drivetrain.turn_for(LEFT, 90, DEGREES)
        else:
            # 3. Check for Orange object
            vision_20.take_snapshot(vision_20__ORANGG)
            if vision_20.object_count > 0:
                # Check object width threshold (100 pixels)
                if vision_20.largest_object.width > 100:
                    drivetrain.drive_for(REVERSE, 200, MM)
                else:
                    block_turn()

def autonomous():
    """15-Second Autonomous Routine for VRC High Stakes / Override"""
    brain.screen.clear_screen()
    brain.screen.print("Running Autonomous...")
    
    # Set default speeds and hold modes
    drivetrain.set_drive_velocity(60, PERCENT)
    drivetrain.set_turn_velocity(40, PERCENT)
    arm.set_stopping(HOLD)
    claw.set_stopping(HOLD)
    
    # STEP 1: Secure starting mobile goal / ring element
    # Close claw on pre-loaded game element
    close_claw()
    wait(0.3, SECONDS)
    
    # Lift arm slightly to clear field tile floor
    arm.spin_to_position(MATCH_LOAD_DEG, DEGREES, wait=True)
    
    # STEP 2: Drive to high-scoring goal zone
    drivetrain.drive_for(FORWARD, 600, MM)
    
    # STEP 3: Vision Sensor Alignment
    # Scan for target orange element/goal
    vision_20.take_snapshot(vision_20__ORANGG)
    
    if vision_20.object_count() > 0:
        # Align horizontal center with screen center (X=160 in a 316x212 frame)
        center_x = vision_20.largest_object().centerX
        if center_x < 140:
            drivetrain.turn_for(LEFT, 10, DEGREES)
        elif center_x > 180:
            drivetrain.turn_for(RIGHT, 10, DEGREES)
            
        # Drive forward to approach goal
        drivetrain.drive_for(FORWARD, 300, MM)
    else:
        # Standard fallback movement if target object is not detected
        drivetrain.drive_for(FORWARD, 300, MM)
        
    # STEP 4: Raise Arm and Score
    arm.spin_to_position(FIRST_STACK_DEG, DEGREES, wait=True)
    
    # Lower claw slightly and open to place ring/goal
    arm_spin_volts(REVERSE, PLACE_VOLTS)
    wait(PLACE_WAIT_SEC, SECONDS)
    open_claw()
    arm.stop()
    
    # STEP 5: Back away to clear scoring zone and zero arm
    drivetrain.drive_for(REVERSE, 400, MM)
    zero_arm()

# when started
arm.set_velocity(100, PERCENT)
arm.set_stopping(HOLD)
claw.set_velocity(CLAW_VELOCITY, PERCENT)
claw.set_stopping(HOLD)
claw.set_position(CLAW_OPEN_DEG, DEGREES)


comp = Competition(user_control, autonomous)