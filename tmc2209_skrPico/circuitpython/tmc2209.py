import busio
import struct
import time

# datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf

class TMC2209:
    """Class for TMC2209 stepper driver communication"""
    def __init__(self, addr, tx, rx, baudrate=115200 , verbose=False):
        self.addr = addr
        self.uart = busio.UART(tx=tx, rx=rx, baudrate=baudrate, bits=8, parity=None, stop=1, timeout=20000/baudrate)
        self.uart.reset_input_buffer()
        self.verbose = verbose

    def deinit(self):
        if self.uart:
            self.uart.deinit()
            self.uart = None

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