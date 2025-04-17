#include <EEPROM.h>
#include <GCodeParser.h>       // https://github.com/tgolla/GCodeParser
#include <Thermistor.h>        // https://github.com/suoapvs/NTC_Thermistor
#include <NTC_Thermistor.h>    // ^^
#include <AverageThermistor.h> // ^^
#include <QuickPID.h>          // https://github.com/Dlloydev/QuickPID
#include "sTune.h"             // https://github.com/Dlloydev/sTune (but modified)

// Firmware Info
#define FIRMWARE_NAME           "Arduino Gcode Interpreter" 
#define FIRMWARE_VERSION        "0.3"
#define SOURCE_CODE_URL         "https://github.com/Zergie"
#define PROTOCOL_VERSION        "1.0"
#define MACHINE_TYPE            "DIY Toaster"
#define UUID                    "554d5e22-d685-4c68-973a-b57ff1e82f6f"
#define TEMP_RESIDENCY_TIME     5  // Actual temperature must be close to target for this long (in seconds) before M109 returns success

// Thermistor
#define SENSOR_PIN1             PA6
#define SENSOR_PIN2             PA0
#define SENSOR_PIN3             PA5
#define REFERENCE_RESISTANCE    4700.0
#define NOMINAL_RESISTANCE      100000.0
#define NOMINAL_TEMPERATURE     25.0
#define BETA_VALUE              3950.0
#define ANALOG_RESOLUTION       4095.0
#define READINGS_NUMBER         10 // How many readings are taken to determine a mean temperature. The more values, the longer a calibration is performed, but the readings will be more accurate.
#define DELAY_TIME              10 //  Delay time between a temperature readings from the temperature sensor (ms).
#define MINIMUM_VALUE           0.0
#define MAXIMUM_VALUE           300.0

// Relay
#define RELAY_PIN               PB9
#define LED_BUILTIN             PC13

uint32_t dwellTime = 0;
double kp = 0; //saved in eeprom
double ki = 0; //saved in eeprom
double kd = 0; //saved in eeprom
float waitTemp = 0;
float waitTempTime = 0;
float targetTemp = 0;
float actualTemp = 0;
float actualTemp1 = 0;
float actualTemp2 = 0;
float actualTemp3 = 0;
float heaterPower = 0;
uint32_t tempAutoReportInterval = 0;
double tempAutoReportTime = 0;
uint32_t tempCheckTime = 0;
float tempCheckTemp = 0;
uint8_t pidTuningEnabled = 0;

// Heater Fault Check
uint32_t heaterCheckTime = 0.0;   // This controls heater verification during initial heating. (in seconds) //saved in eeprom
float heaterTempGain = 0.0;       // The minimum temperature (in Celsius) that the heater must increase by during the check_gain_time check. //saved in eeprom
float heaterTempHysteresis = 0.0; // The maximum temperature difference (in Celsius) to a target temperature that is considered in range of the target. //saved in eeprom
float heaterTempRange[6] = { 0.0, 300.0, 0.0, 300.0, 0.0, 80.0 }; // {min, max, min, max, ...}
//float heaterTempRange[6] = { -300.0, 300.0, -300.0, 300.0, -300.0, 80.0 }; // {min, max, min, max, ...}

HardwareSerial Serial = HardwareSerial(PA10, PA9);
GCodeParser GCode = GCodeParser();
Thermistor* thermistor1;
Thermistor* thermistor2;
Thermistor* thermistor3;
QuickPID pid = QuickPID(&actualTemp, &heaterPower, &targetTemp);
HardwareTimer *MyTim = new HardwareTimer(TIM2);
sTune tuner = sTune(&actualTemp, &heaterPower, tuner.ZN_PID, tuner.directIP, tuner.printOFF);

void saveToEEPROM() {
  int address = 0;
  EEPROM.write(address, kp); address += sizeof(double);
  EEPROM.write(address, ki); address += sizeof(double);
  EEPROM.write(address, kd); address += sizeof(double);
  EEPROM.write(address, heaterCheckTime); address += sizeof(uint32_t);
  EEPROM.write(address, heaterTempGain); address += sizeof(float);
  EEPROM.write(address, heaterTempHysteresis); address += sizeof(float);
}

void loadFromEEPROM() {
  int address = 0;
  kp                   = EEPROM.read(address); address += sizeof(double);
  ki                   = EEPROM.read(address); address += sizeof(double);
  kd                   = EEPROM.read(address); address += sizeof(double);
  heaterCheckTime      = EEPROM.read(address); address += sizeof(uint32_t);
  heaterTempGain       = EEPROM.read(address); address += sizeof(float);
  heaterTempHysteresis = EEPROM.read(address); address += sizeof(float);

  pid.SetTunings(kp, ki, kd);
}

void Update_IT_callback(void)
{
  if (heaterPower >= 1.0) {
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
  }
}
void Compare_IT_callback(void)
{
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  thermistor1 = new NTC_Thermistor(
    SENSOR_PIN1,
    REFERENCE_RESISTANCE,
    NOMINAL_RESISTANCE,
    NOMINAL_TEMPERATURE,
    BETA_VALUE,
    ANALOG_RESOLUTION
  );
  thermistor2 = new NTC_Thermistor(
    SENSOR_PIN2,
    REFERENCE_RESISTANCE,
    NOMINAL_RESISTANCE,
    NOMINAL_TEMPERATURE,
    BETA_VALUE,
    ANALOG_RESOLUTION
  );
  thermistor3 = new NTC_Thermistor(
    SENSOR_PIN3,
    REFERENCE_RESISTANCE,
    NOMINAL_RESISTANCE,
    NOMINAL_TEMPERATURE,
    BETA_VALUE,
    ANALOG_RESOLUTION
  );
  loadFromEEPROM();
  pid.SetOutputLimits(0, 255);
  pid.SetMode(pid.Control::automatic);
  MyTim->setOverflow(10000, MICROSEC_FORMAT);
  MyTim->attachInterrupt(Update_IT_callback);
  MyTim->setCaptureCompare(1, 0, RESOLUTION_8B_COMPARE_FORMAT);
  MyTim->attachInterrupt(1, Compare_IT_callback);
  MyTim->resume();
  while(!Serial);
}

void readTemps() {
  actualTemp1 = thermistor1->readCelsius();
  actualTemp2 = thermistor2->readCelsius();
  actualTemp3 = thermistor3->readCelsius();
  actualTemp = (actualTemp1 + actualTemp2) / 2;
}

void setHeater() {
  MyTim->setCaptureCompare(1, (uint32_t)heaterPower, RESOLUTION_8B_COMPARE_FORMAT);
  MyTim->resume();
}

void processCommandG(int codeNumber) {
  switch (codeNumber) {
    case 4: // Dwell
      if (GCode.HasWord('S')) { 
        dwellTime = millis() + (int)GCode.GetWordValue('S') * 1000;
      } else if (GCode.HasWord('P')) { 
        dwellTime = millis() + (int)GCode.GetWordValue('P');
      }
      break;
    default:
      Serial.print("Unknown G command!\n");
      break;
  }
}

void processCommandM(int codeNumber) {
  switch (codeNumber) {
    case 1: // Test
      break;
    case 2: // Program End
      HAL_NVIC_SystemReset();
      break;
    case 104: // Set Hotend Temperature
      if (GCode.HasWord('S')) { targetTemp = (int)GCode.GetWordValue('S'); }
      tempCheckTime = millis();
      Serial.print("ok\n");
      break;
    case 105: // Report Temperatures
      Serial.print("T0:");
      Serial.print(actualTemp);
      Serial.print(" /");
      Serial.print((int)targetTemp);
      Serial.print(" @:");
      Serial.print((int)heaterPower);
      Serial.print(" T1:");
      Serial.print(actualTemp1);
      Serial.print(" T2:");
      Serial.print(actualTemp2);
      Serial.print(" T3:");
      Serial.print(actualTemp3);
      Serial.print("\n");
      break;
    // case 108: // Cancel Heating //Breaks out of an M109 wait-for-temperature loop
    //   targetTemp = 0;
    //   waitTemp = 0;
    //   break;
    case 109: // Wait for Hotend Temperature
      if (GCode.HasWord('S')) { targetTemp = (int)GCode.GetWordValue('S'); }
      waitTemp = targetTemp;
      tempCheckTime = millis();
      break;
    case 115: // Firmware Info
      Serial.print("FIRMWARE_NAME: ");
      Serial.print(FIRMWARE_NAME);
      Serial.print(" v");
      Serial.print(FIRMWARE_VERSION);
      Serial.print(" SOURCE_CODE_URL: ");
      Serial.print(SOURCE_CODE_URL);
      Serial.print(" PROTOCOL_VERSION: ");
      Serial.print(PROTOCOL_VERSION);
      Serial.print(" MACHINE_TYPE: ");
      Serial.print(MACHINE_TYPE);
      Serial.print(" UUID: ");
      Serial.print(UUID);
      Serial.print("\n");
      break;
    case 130: // Set PID P Value
      if (GCode.HasWord('S')) { kp = GCode.GetWordValue('S'); }
      pid.SetTunings(kp, ki, kd);
      Serial.print("ok\n");
      break;
    case 131: // Set PID I Value
      if (GCode.HasWord('S')) { ki = GCode.GetWordValue('S'); }
      pid.SetTunings(kp, ki, kd);
      Serial.print("ok\n");
      break;
    case 132: // Set PID D Value
      if (GCode.HasWord('S')) { kd = GCode.GetWordValue('S'); }
      pid.SetTunings(kp, ki, kd);
      Serial.print("ok\n");
      break;
    case 155: // Temperature Auto-Report
      if (GCode.HasWord('S')) { tempAutoReportInterval = (float)GCode.GetWordValue('S') * 1000; }
      Serial.print("ok\n");
      break;
    case 301: // Set Hotend PID
      if (GCode.HasWord('P')) { kp = GCode.GetWordValue('P'); }
      if (GCode.HasWord('I')) { ki = GCode.GetWordValue('I'); }
      if (GCode.HasWord('D')) { kd = GCode.GetWordValue('D'); }
      pid.SetTunings(kp, ki, kd);
      Serial.print("ok\n");
      break;
    case 303: // Run PID tuning // not working
      if (!GCode.HasWord('S')) { 
        Serial.print("Parameter S is required!\n");
      } else if (!GCode.HasWord('C')) { 
        Serial.print("Parameter C is required!\n");
      } else {
        tuner.Configure(heaterTempRange[1] - heaterTempRange[0], 255, 0, 1, 500, 10, 500);
        tuner.SetEmergencyStop(heaterTempRange[1]);
        targetTemp = (int)GCode.GetWordValue('S');
        pidTuningEnabled = (int)GCode.GetWordValue('C');
        tempCheckTime = millis();
      }
      break;
    case 500: // Store current settings to EEPROM
      saveToEEPROM();
      Serial.print("ok\n");
      break;
    case 501: // Read all parameters from EEPROM
      loadFromEEPROM();
      Serial.print("ok\n");
      break;
    case 502: // Restore current settings to defaults
      // https://insideautomation.net/initial-settings-pid-controllers/
      kp = 25.0;
      ki = 1.0;
      kd = 110.0;
      heaterCheckTime = 20;
      heaterTempGain = 1.0;
      heaterTempHysteresis = 10.0;
      Serial.print("ok\n");
      break;
    case 503: // Print the current settings
      Serial.print("kp: "); Serial.print(kp); Serial.print("\n");
      Serial.print("ki: "); Serial.print(ki); Serial.print("\n");
      Serial.print("kd: "); Serial.print(kd); Serial.print("\n");
      Serial.print("Heater Check Time: "); Serial.print(heaterCheckTime); Serial.print("\n");
      Serial.print("Heater Temperatur Gain: "); Serial.print(heaterTempGain); Serial.print("\n");
      Serial.print("Heater Temperatur Hysteresis: "); Serial.print(heaterTempHysteresis); Serial.print("\n");
      Serial.print("Heater Temperatur Range 0: "); Serial.print(heaterTempRange[0]); Serial.print(" - "); Serial.print(heaterTempRange[1]); Serial.print("\n");
      Serial.print("Heater Temperatur Range 1: "); Serial.print(heaterTempRange[2]); Serial.print(" - "); Serial.print(heaterTempRange[3]); Serial.print("\n");
      Serial.print("Heater Temperatur Range 2: "); Serial.print(heaterTempRange[4]); Serial.print(" - "); Serial.print(heaterTempRange[5]); Serial.print("\n");
      break;
    case 570: // Configure heater fault detection (custom gcode!)
      if (GCode.HasWord('P')) { heaterCheckTime = GCode.GetWordValue('P'); }
      if (GCode.HasWord('T')) { heaterTempGain = GCode.GetWordValue('T'); }
      if (GCode.HasWord('S')) { heaterTempHysteresis = GCode.GetWordValue('S'); }
      Serial.print("ok\n");
      break;
    case 571: // Configure heater range (custom gcode!)
      if (!GCode.HasWord('H')) { 
        Serial.print("Parameter H (heater) is required!\n");
      } else {
        uint32_t heater = 0;
        if (GCode.HasWord('H')) { heater = GCode.GetWordValue('H'); }
        if (GCode.HasWord('T')) { heaterTempRange[heater * 2] = GCode.GetWordValue('T'); } // min
        if (GCode.HasWord('P')) { heaterTempRange[(heater * 2)+1] = GCode.GetWordValue('T'); } // max
        Serial.print("ok\n");
      }
      break;
    default:
      Serial.print("Unknown M command!\n");
      break;
  }
}

void termalProtection(uint32_t time) {
  if (actualTemp1 < heaterTempRange[0] || actualTemp1 > heaterTempRange[1]) {
    heaterFault();
    Serial.print("ERROR: Termister 0 temperature out of bounds! (");
    Serial.print(heaterTempRange[0]);
    Serial.print(" <= ");
    Serial.print(actualTemp1);
    Serial.print(" <= ");
    Serial.print(heaterTempRange[1]);
    Serial.print(")\n");
  }
  if (actualTemp2 < heaterTempRange[2] || actualTemp2 > heaterTempRange[3]) {
    heaterFault();
    Serial.print("ERROR: Termister 1 temperature out of bounds! (");
    Serial.print(heaterTempRange[2]);
    Serial.print(" <= ");
    Serial.print(actualTemp2);
    Serial.print(" <= ");
    Serial.print(heaterTempRange[3]);
    Serial.print(")\n");
  }
    if (actualTemp3 < heaterTempRange[4] || actualTemp3 > heaterTempRange[5]) {
    heaterFault();
    Serial.print("ERROR: Termister 2 temperature out of bounds! (");
    Serial.print(heaterTempRange[4]);
    Serial.print(" <= ");
    Serial.print(actualTemp3);
    Serial.print(" <= ");
    Serial.print(heaterTempRange[5]);
    Serial.print(")\n");
  }

  if ((time - tempCheckTime) > heaterCheckTime * 1000) {
    if (targetTemp == 0) {
      // this is considered off, so no heating
    } else if (abs(actualTemp - targetTemp) <= heaterTempHysteresis) {
      // this is considered on target
    } else if (tempCheckTime == 0) {
      // wait for one more iteration to get the temp gain
    } else {
      float gainTemp = actualTemp - tempCheckTemp;
      if (actualTemp < targetTemp && gainTemp < heaterTempGain) {
        heaterFault();
        Serial.print("ERROR: Heater not heating at expected rate!\n");
      }
    }
    tempCheckTemp = actualTemp;
    tempCheckTime = time;
  }
}

void heaterFault() {
  heaterPower = 0;
  targetTemp = 0;
  waitTemp = 0;
}

void loop() {
  // Serial.print(".");
  uint32_t time = millis();
  readTemps();

  termalProtection(time);

  if (tempAutoReportInterval > 0) {
    if (tempAutoReportTime == 0 || (time - tempAutoReportTime) > tempAutoReportInterval) {
      processCommandM(105);
      tempAutoReportTime = time;
    }
  }

  if (pidTuningEnabled) {
    setHeater();

    switch (tuner.Run()) {
      case tuner.sample: // active once per sample during test
        readTemps();
        break;
      
      case tuner.runPid: // active once per sample after tunings
        readTemps();
        pid.Compute();
        break;

      case tuner.tunings: // active just once when sTune is done
        kp = tuner.GetKp();
        ki = tuner.GetKi();
        kd = tuner.GetKd();
        Serial.print("Kp: ");
        Serial.print(kp);
        Serial.print(" Ki: ");
        Serial.print(ki);
        Serial.print(" Kd: ");
        Serial.print(kd);
        Serial.print("\n");
        Serial.print("PID Autotune finished!\n");
        pidTuningEnabled--;
        heaterPower = 0;
        targetTemp = 0;
        break;
    }
  } else {
    pid.Compute();
    setHeater();

    if (dwellTime > 0) {
      if (time > dwellTime) {
        dwellTime = 0;
        Serial.print("ok\n");
      }
    } else if (waitTemp > 0) {
      if (actualTemp < waitTemp) {
        waitTempTime = 0;
      } else if (waitTempTime == 0) {
        waitTempTime = time;
      } else if (time - waitTempTime >= TEMP_RESIDENCY_TIME * 1000) {
        waitTemp = 0;
        waitTempTime = 0;
        Serial.print("ok\n");
      }
    } else if (!Serial.available() > 0 || !GCode.AddCharToLine(Serial.read()) || GCode.blockDelete) {
    //pass 
    } else if (GCode.HasWord('M')) {
      processCommandM((int)GCode.GetWordValue('M'));
    } else if (GCode.HasWord('G')) {
      processCommandG((int)GCode.GetWordValue('G'));
    } else {
      Serial.print("Unknown command!\n");
    }
  }
}
