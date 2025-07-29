import busio
import digitalio
import struct
import time
import microcontroller
import pwmio
import pulseio
import array
import rp2pio
import adafruit_pioasm

# datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf

class TMC2209:
    STEP_BUFFER_SIZE = 1024 # Number of steps to buffer for each move

    """Class for TMC2209 stepper driver communication"""
    def __init__(self, addr, tx, rx, en, step, dir, baudrate=115200, verbose=False):
        self.verbose = verbose
        self.addr = addr
        self.uart = busio.UART(tx=tx, rx=rx, baudrate=baudrate, bits=8, parity=None, stop=1, timeout=20000/baudrate)
        self.uart.reset_input_buffer()
        
        self.en_pin = digitalio.DigitalInOut(en)
        self.dir_pin = digitalio.DigitalInOut(dir)

        self.en_pin.direction = digitalio.Direction.OUTPUT
        self.dir_pin.direction = digitalio.Direction.OUTPUT

        self.en_pin.value = True  # Disable stepper driver initially
        self.dir_pin.value = True  # Set initial direction to CW

        self._is_enabled = False
        self._direction = 1

        self._pio = None
        self._step = step
        self.step_pin = None
    
    def __del__(self):
        if self.uart:
            self.uart.deinit()
            self.uart = None
        self.disable()  # Ensure the driver is disabled on deletion

    def enable(self):
        self.en_pin.value = False
        self._is_enabled = True

    def disable(self):
        self.en_pin.value = True
        self._is_enabled = False

    def _setup_move(self, speed):
        if not self._is_enabled:
            self.enable()

        if speed < 0:
            direction = -1
            speed = -speed
        else:
            direction = 1
        
        if direction != self._direction:
            self.dir_pin.value = not self.dir_pin.value 
            self._direction = direction
        return speed

    def move(self, steps, speed, acceleration=1000, iterations=10):
        speed = self._setup_move(speed)
        
        if not self.step_pin is None:
            self.step_pin.deinit()
            self.step_pin = None

        if self._pio is None:
            self._pio = rp2pio.StateMachine(
                adafruit_pioasm.assemble("""
.program step
    out x, 32         ; fetch delay
    out y, 32         ; fetch steps
loop:
    set pins, 1 [20]  ; Turn STEP on (100ns pulse)
    set pins, 0       ; Turn STEP off
    mov isr, x        ; Move delay to ISR
delay:
    jmp x-- delay     ; Delay for x cycles
    mov x, isr        ; Restore x from ISR
    jmp y-- loop      ; Loop until all steps are done
"""),
            frequency=microcontroller.cpu.frequency // 4,
            auto_pull=True,
            first_set_pin=self._step,
        )
        
        # Generate acceleration profile (trapezoidal)
        # accceleration: steps/sec^2, speed: max steps/sec, steps: total steps
        # Calculate ramp up/down steps
        if acceleration == 0:
            ramp_steps = 0
            flat_steps = steps
        else:
            ramp_steps = int(speed**2 / (2 * acceleration * iterations))
            ramp_steps = min(ramp_steps, steps // 2)
            flat_steps = steps - 2 * ramp_steps

        print(f"Ramp steps: {ramp_steps}, Flat steps: {flat_steps}, Total steps: {steps}, Speed: {speed}, Acceleration: {acceleration}")
        
        # Stream step data to PIO in chunks to avoid memory issues
        def step_profile():

            # Ramp up
            for i in range(1, ramp_steps, iterations):
                current_speed = int((acceleration * (i + 1))**0.5)
                current_speed = min(current_speed, speed)
                delay = max(1, self._pio.frequency // current_speed)
                yield delay
                yield iterations - 1

            # Constant speed
            if flat_steps > 0:
                delay = max(1, self._pio.frequency // speed)
                yield delay
                yield flat_steps - 1

            # Ramp down
            for i in range(ramp_steps, 1, -iterations):
                current_speed = int((acceleration * i)**0.5)
                current_speed = min(current_speed, speed)
                delay = max(1, self._pio.frequency // current_speed)
                yield delay
                yield iterations - 1

        # Stream data in buffer-sized chunks
        buf = array.array("I")
        count = 0
        for value in step_profile():
            buf.append(value)
            count += 1
            if count >= self.STEP_BUFFER_SIZE:
                self._pio.background_write(buf)
                buf = array.array("I")
                count = 0
        if count > 0:
            self._pio.write(buf)

    def run(self, speed, acceleration=1000):
        speed = self._setup_move(speed)
        
        if not self._pio is None:
            self._pio.deinit()
            self._pio = None

        if self.step_pin is None:
            self.step_pin = pwmio.PWMOut(self._step, frequency=50, duty_cycle=0, variable_frequency=True)

        # Generate acceleration profile
        if acceleration == 0 or speed == 0:
            ramp_steps = 0
            ramp_time = 0
        else:
            ramp_time = speed // acceleration // 1000
            ramp_steps = max(1, int(ramp_time / 0.01))
        
        def set_speed(speed, duration):
            print(f"Setting speed: {int(speed)} steps/s, Duration: {duration if duration >= 0 else 'constant'}")
            self.step_pin.frequency = int(speed)
            self.step_pin.duty_cycle = min(max(int((100e-9 / (1 / self.step_pin.frequency)) * 65535), 1), 65535)
            if duration > 0:
                time.sleep(duration)

        # Ramp up (ease-in: accelerate quickly at first, then slower)
        for i in range(1, ramp_steps):
            # Use a square root profile for acceleration (ease-in)
            factor = (i / ramp_steps) ** 0.5
            set_speed(speed * factor, ramp_time / ramp_steps)
        
        # Constant speed
        set_speed(speed, -1)


    def _print(self, message):
        if self.verbose:
            print(message)

    def compute_crc8_atm(self, datagram, initial_value=0):
        crc = initial_value
        # Iterate bytes in data
        for byte in datagram:
            # Iterate bits in byte
            for _ in range(0, 8):
                if (crc >> 7) ^ (byte & 0x01):
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
                # Shift to next bit
                byte = byte >> 1
        return crc
    

    
    def read_register(self, register):
        r_frame = [0x55, 0, 0, 0 ]
        r_frame[1] = self.addr
        r_frame[2] = register  # Register to read
        r_frame[3] = self.compute_crc8_atm(r_frame[:-1])
        b = struct.pack('>BBBB', *r_frame)
        self._print(f"> sending: {b}")
        self.uart.reset_input_buffer()  # Clear any previous data
        self.uart.write(b)

        # Discard echo
        time.sleep(0.005)
        self.uart.read(4)

        # Wait for response
        time.sleep(0.005)
        response = self.uart.read(8)
        self._print(f"< received: {response}")
        if response is None or len(response) < 8:
            raise RuntimeError(f"No response or too short: {response} {len(response) if response else 0} < 8")

        return struct.unpack('>I', response[3:7])[0]

    def read_IOIN(self):
        response = self.read_register(0x06)
        return {
            "ENN": (response >> 0) & 0x01,
            "MS1" : (response >> 2) & 0x01,
            "MS2" : (response >> 3) & 0x01,
            "DIAG" : (response >> 4) & 0x01,
            "PDN_UART" : (response >> 6) & 0x01,
            "STEP" : (response >> 7) & 0x01,
            "SPREAD_EN" : (response >> 8) & 0x01,
            "DIR" : (response >> 9) & 0x01, 
            "VERSION" : (response >> 24) & 0xFF
        }
    def read_IFCNT(self):
        return self.read_register(0x02)
        

    def write_register(self, register, value):
        w_frame = [0x55, 0, 0, 0 , 0, 0, 0, 0 ]
        w_frame[1] = self.addr
        w_frame[2] = register | 0x80  # set write bit
        w_frame[3] = 0xFF & (value>>24)
        w_frame[4] = 0xFF & (value>>16)
        w_frame[5] = 0xFF & (value>>8)
        w_frame[6] = 0xFF & value
        w_frame[7] = self.compute_crc8_atm(w_frame[:-1])
        b = struct.pack('>BBBBBBBB', *w_frame)
        self._print(f"> sending: {b}")
        self.uart.write(b)
    
    def write_GCONF(self, I_scale_analog=0, internal_Rsense=0, en_SpreadCycle=0, index_otpw=0, index_step=0, pdn_disable=0, mstep_reg_select=0, multistep_filt=0, test_mode=0):  
        self.write_register(0x00, 
            I_scale_analog   << 0 |
            internal_Rsense  << 1 |
            en_SpreadCycle   << 2 |
            index_otpw       << 4 |
            index_step       << 5 |
            pdn_disable      << 6 |
            mstep_reg_select << 7 |
            multistep_filt   << 8 |
            test_mode        << 9 )
    
    def write_CHOPCONF(self, toff=0, hstrt=0, hend=0, disfdcc=0, rndtf=0, chm=0, tbl=0, mres=0, intpol=0, microsteps=0):
        if (microsteps > 0):
            if (mres != 0):
                raise ValueError("mres must be 0 if microsteps is set")
            lookup = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8}
            if microsteps not in lookup:
                raise ValueError(f"Invalid microsteps value: {microsteps}. Must be one of {list(lookup.keys())}")
            mres = lookup[microsteps]
        
        self.write_register(0x6C, 
            toff       << 0 |
            hstrt      << 3 |
            hend       << 7 |
            disfdcc    << 10 |
            rndtf      << 11 |
            chm        << 12 |
            tbl        << 13 |
            mres       << 24 |
            intpol     << 28)

    def write_IHOLD_IRUN(self, ihold=0, irun=0, iholddelay=0):
        self.write_register(0x10, 
            ihold       << 0 |
            irun        << 8 |
            iholddelay  << 16)

    def write_PWMCONF(self, PWM_LIM=12, PWM_REG=0, freewheel=0, pwm_autograd=0, pwm_autoscale=0, pwm_freq=0, PWM_GRAD=0, PWM_OFS=36):
        self.write_register(0x70, 
            PWM_OFS       << 0 |
            PWM_GRAD      << 8 |
            pwm_freq      << 16 |
            pwm_autoscale << 18 |
            pwm_autograd  << 19 |
            freewheel     << 20 |
            PWM_REG       << 24 |
            PWM_LIM       << 28)