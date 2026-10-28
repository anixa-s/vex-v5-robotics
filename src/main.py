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








# when started
arm.set_velocity(100, PERCENT)
arm.set_stopping(HOLD)
claw.set_velocity(CLAW_VELOCITY, PERCENT)
claw.set_stopping(HOLD)
claw.set_position(CLAW_OPEN_DEG, DEGREES)


comp = Competition(user_control, autonomous)