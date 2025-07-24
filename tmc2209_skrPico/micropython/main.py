from tmc.TMC_2209_StepperDriver import *

class SkrPico:
    def __init__(self):
        self._neopixel = None
        self._stepperX = None
        self._stepperY = None
        self._stepperZ = None
        self._stepperE = None
        pass
    
    def getNeopixel(self, n:int=1):
        if self._neopixel == None:
            import neopixel
            self._neopixel = neopixel.NeoPixel(machine.Pin(24), n)
        return self._neopixel

    def getStepperX(self):
        if self._stepperX == None:
            import tmc.TMC_2209_StepperDriver
            self._stepperX = tmc.TMC_2209_StepperDriver.TMC_2209(11, 10, 12, 0)
        return self._stepperX
    def getStepperY(self):
        if self._stepperY == None:
            import tmc.TMC_2209_StepperDriver
            self._stepperY = tmc.TMC_2209_StepperDriver.TMC_2209( 6,  5,  7, 2)
        return self._stepperY
    def getStepperZ(self):
        if self._stepperZ == None:
            import tmc.TMC_2209_StepperDriver
            self._stepperZ = tmc.TMC_2209_StepperDriver.TMC_2209(19, 28, 28, 1)
        return self._stepperZ
    def getStepperE(self):
        if self._stepperE == None:
            import tmc.TMC_2209_StepperDriver
            self._stepperE = tmc.TMC_2209_StepperDriver.TMC_2209(14, 13, 15, 3)
        return self._stepperE


print("---")
print("SCRIPT START")
print("---")
#-----------------------------------------------------------------------
# initiate the TMC_2209 class
#-----------------------------------------------------------------------
mcu = SkrPico()
tmc = mcu.getStepperX()
#-----------------------------------------------------------------------
# set the loglevel of the libary (currently only printed)
# set whether the movement should be relative or absolute
# both optional
#-----------------------------------------------------------------------
tmc.setLoglevel(Loglevel.debug)
tmc.setMovementAbsRel(MovementAbsRel.relative)
#-----------------------------------------------------------------------
# these functions change settings in the TMC register
#-----------------------------------------------------------------------
tmc.setDirection_reg(False)
tmc.setVSense(True)
tmc.setCurrent(800)
tmc.setIScaleAnalog(True)
tmc.setInterpolation(True)
tmc.setSpreadCycle(False)
tmc.setMicrosteppingResolution(2)
tmc.setInternalRSense(False)
print("---\n---")
#-----------------------------------------------------------------------
# these functions read and print the current settings in the TMC register
#-----------------------------------------------------------------------
tmc.readIOIN()
tmc.readCHOPCONF()
tmc.readDRVSTATUS()
tmc.readGCONF()
print("---\n---")
#-----------------------------------------------------------------------
# set the Accerleration and maximal Speed
#-----------------------------------------------------------------------
tmc.setAcceleration(50000)
tmc.setMaxSpeed(320000)
#-----------------------------------------------------------------------
# move the motor
#-----------------------------------------------------------------------
tmc.setMotorEnabled(True)
# tmc.runToPositionSteps(400)                             #move to position 400
# tmc.runToPositionSteps(0)                               #move to position 0
# tmc.runToPositionSteps(+400, MovementAbsRel.relative)   #move forward
# tmc.runToPositionSteps(-400, MovementAbsRel.relative)   #move backward
# tmc.runToPositionSteps(400)                             #move to position 400
# tmc.runToPositionSteps(0)                               #move to position 0
tmc.runToPositionSteps(+32000, MovementAbsRel.relative)    #move forward
tmc.setMotorEnabled(False)
print("---\n---")
#-----------------------------------------------------------------------
# deinitiate the TMC_2209 class
#-----------------------------------------------------------------------
del tmc
del mcu
print("---")
print("SCRIPT FINISHED")
print("---")