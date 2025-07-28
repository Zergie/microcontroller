import board
import microcontroller
import time

from tmc2209 import TMC2209

# Constants for stepper motor control
STEPS_PER_REV = 200  # Steps per revolution for the stepper motor

# Adjust these to your wiring
TMC2209_ADDR = 0  # X-Stepper on SKR Pico
TX_PIN       = board.GP8   # UART TX pin
RX_PIN       = board.GP9   # UART RX pin
EN_PIN       = board.GP12  # Enable pin for the stepper driver
STEP_PIN     = board.GP11  # Step pin for the stepper driver
DIR_PIN      = board.GP10  # Direction pin for the stepper driver

### Step 1: Initialize microcontroller
microcontroller.cpu.frequency = 200000000  # Set CPU frequency to 2 MHz
print(f"Microcontroller: {microcontroller.cpu.frequency / 1000 / 1000} MHz")


### Step 2: Initialize TMC2209 
tmc = TMC2209(addr=TMC2209_ADDR, tx=TX_PIN, rx=RX_PIN, en=EN_PIN, step=STEP_PIN, dir=DIR_PIN)
print(f"IOIN: {tmc.read_IOIN()}")
print(f"IFCNT: {tmc.read_IFCNT()}")
tmc.write_GCONF(pdn_disable=1, mstep_reg_select=1, multistep_filt=1)
tmc.write_IHOLD_IRUN(ihold=5, irun=26, iholddelay=1)

### Step 3: wait for serial connection
# while 1:
#     print("Press Enter to continue after connecting to the serial console...")
#     i, o, e = select.select( [sys.stdin], [], [], 1 )
#     if i:
#         print("")
#         print("Serial connection established.")
#         print("")
#         break

microsteps = 64
print(f"Setting microsteps to {microsteps}")
tmc.write_CHOPCONF(toff=5, microsteps=microsteps, intpol=1)

speed = 100000 #128000 * 3
for _ in range(2):
    steps = []

    steps = STEPS_PER_REV * microsteps
    tmc.move(steps=steps, speed=speed,)
    print(f"Steps: {steps}, Speed: {speed}")
    # tmc.run(speed=speed)

    
    speed += 25000
    time.sleep(1)

tmc.disable()
print("Done stepping")
input("Press Enter to exit...")
