import board
import time
import digitalio

from tmc2209 import TMC2209

# Constants for stepper motor control
MICROSTEPS    = 2  # Set the number of microsteps (1, 2, 4, 8, 16, 32, 64, 128, 256)
STEP_TIME     = 1 / 1900  # Time per step in seconds (adjust as needed)
STEPS_PER_REV = 200  # Steps per revolution for the stepper motor

# Adjust these to your wiring
TMC2209_ADDR = 0  # X-Stepper on SKR Pico
TX_PIN       = board.GP8   # UART TX pin
RX_PIN       = board.GP9   # UART RX pin
EN_PIN       = board.GP12  # Enable pin for the stepper driver
STEP_PIN     = board.GP11  # Step pin for the stepper driver
DIR_PIN      = board.GP10  # Direction pin for the stepper driver

# Adjust these to your wiring
en_pin = digitalio.DigitalInOut(EN_PIN)
en_pin.direction = digitalio.Direction.OUTPUT

step_pin = digitalio.DigitalInOut(STEP_PIN)
step_pin.direction = digitalio.Direction.OUTPUT

dir_pin = digitalio.DigitalInOut(DIR_PIN)
dir_pin.direction = digitalio.Direction.OUTPUT

### Step 2: Initialize UART
tmc = TMC2209(addr=TMC2209_ADDR, tx=TX_PIN, rx=RX_PIN, verbose=True)

### Step 3: Verify communication
# response = tmc.read_register(0x06)
print(f"IOIN: {tmc.read_IOIN()}")

response = tmc.read_register(0x02)
print(f"IFCNT: {tmc.read_IFCNT()}")

# Set microstepping
tmc.write_GCONF(index_step=1, pdn_disable=1, mstep_reg_select=1, multistep_filt=1)
tmc.write_IHOLD_IRUN(ihold=5, irun=20, iholddelay=1)
tmc.write_CHOPCONF(toff=5, microsteps=MICROSTEPS, intpol=1)

tmc.deinit()  # Close UART connection

en_pin.value = False
dir_pin.value = True

for i in range(STEPS_PER_REV * 2 * MICROSTEPS):
    step_pin.value = True if i % 2 == 0 else False  # Toggle step pin
    time.sleep(STEP_TIME) 
en_pin.value = True  # Disable stepper driver
print("Done stepping")
