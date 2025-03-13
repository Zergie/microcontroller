#include <EEPROM.h>
#include <GCodeParser.h>       // https://github.com/tgolla/GCodeParser
#include <Thermistor.h>        // https://github.com/suoapvs/NTC_Thermistor
#include <NTC_Thermistor.h>    // ^^
#include <AverageThermistor.h> // ^^
#include <QuickPID.h>          // https://github.com/Dlloydev/QuickPID

// Firmware Info
#define FIRMWARE_NAME           "Arduino Gcode Interpreter" 
#define FIRMWARE_VERSION        "0.1"
#define SOURCE_CODE_URL         "https://github.com/Zergie"
#define PROTOCOL_VERSION        "1.0"
#define MACHINE_TYPE            "DIY Toaster"
#define UUID                    "554d5e22-d685-4c68-973a-b57ff1e82f6f"
#define TEMP_RESIDENCY_TIME     5  // Actual temperature must be close to target for this long (in seconds) before M109 returns success

//todo: ?
#define TEMP_HYSTERESIS         5    // The maximum temperature difference (in Celsius) to a target temperature that is considered in range of the target.
#define TEMP_CHECK_TIME         20   // This controls heater verification during initial heating. (in seconds)
#define TEMP_HEATING_GAIN       2.0  // The minimum temperature (in Celsius) that the heater must increase by during the check_gain_time check.
//todo: ?

// Thermistor
#define SENSOR_PIN1              PA6
#define SENSOR_PIN2              PA0
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
float heaterPower = 0;
double tempAutoReportInterval = 0;
double tempAutoReportTime = 0;
uint32_t tempCheckTime = 0;
float tempCheckTemp = 0;
uint8_t pidEnabled = 1;

HardwareSerial Serial = HardwareSerial(PA10, PA9);
GCodeParser GCode = GCodeParser();
Thermistor* thermistor1;
Thermistor* thermistor2;
QuickPID pid = QuickPID(&actualTemp, &heaterPower, &targetTemp);

HardwareTimer *MyTim = new HardwareTimer(TIM2);
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
  Thermistor* originThermistor1 = new NTC_Thermistor(
    SENSOR_PIN1,
    REFERENCE_RESISTANCE,
    NOMINAL_RESISTANCE,
    NOMINAL_TEMPERATURE,
    BETA_VALUE,
    ANALOG_RESOLUTION
  );
  thermistor1 = new AverageThermistor(
    originThermistor1,
    READINGS_NUMBER,
    DELAY_TIME
  );
  Thermistor* originThermistor2 = new NTC_Thermistor(
    SENSOR_PIN2,
    REFERENCE_RESISTANCE,
    NOMINAL_RESISTANCE,
    NOMINAL_TEMPERATURE,
    BETA_VALUE,
    ANALOG_RESOLUTION
  );
  thermistor2 = new AverageThermistor(
    originThermistor2,
    READINGS_NUMBER,
    DELAY_TIME
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

void saveToEEPROM() {
  int address = 0;
  EEPROM.write(address, kp); address += sizeof(double);
  EEPROM.write(address, ki); address += sizeof(double);
  EEPROM.write(address, kd); address += sizeof(double);
}

void loadFromEEPROM() {
  int address = 0;
  kp = EEPROM.read(address); address += sizeof(double);
  ki = EEPROM.read(address); address += sizeof(double);
  kd = EEPROM.read(address); address += sizeof(double);
  pid.SetTunings(kp, ki, kd);
}

void computePid() {
  // read from inputs
  actualTemp1 = thermistor1->readCelsius();
  actualTemp2 = thermistor2->readCelsius();
  actualTemp = (actualTemp1 + actualTemp2) / 2;

  // calculate
  if (pidEnabled) {
    pid.Compute();
  }
  
  // write to 
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
      if (GCode.HasWord('S')) { heaterPower = (int)GCode.GetWordValue('S'); }
      tempAutoReportInterval = 1000;
      pidEnabled = 0;
      break;
    case 2: // Program End
      NVIC_SystemReset();
      break;
    case 104: // Set Hotend Temperature
      if (GCode.HasWord('S')) { targetTemp = (int)GCode.GetWordValue('S'); }
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
      Serial.print(" /");
      Serial.print((int)targetTemp);
      Serial.print(" T2:");
      Serial.print(actualTemp2);
      Serial.print(" /");
      Serial.print((int)targetTemp);
      Serial.print("\n");
      break;
    // case 108: // Cancel Heating //Breaks out of an M109 wait-for-temperature loop
    //   targetTemp = 0;
    //   waitTemp = 0;
    //   break;
    case 109: // Wait for Hotend Temperature
      if (GCode.HasWord('S')) { targetTemp = (int)GCode.GetWordValue('S'); }
      waitTemp = targetTemp;
      break;
    case 115: // Firmware Info
      Serial.print("FIRMWARE_NAME:");
      Serial.print(FIRMWARE_NAME);
      Serial.print(" v");
      Serial.print(FIRMWARE_VERSION);
      Serial.print(" SOURCE_CODE_URL:");
      Serial.print(SOURCE_CODE_URL);
      Serial.print(" PROTOCOL_VERSION:");
      Serial.print(PROTOCOL_VERSION);
      Serial.print(" MACHINE_TYPE:");
      Serial.print(MACHINE_TYPE);
      Serial.print(" UUID:");
      Serial.print(UUID);
      Serial.print("\n");
      break;
    case 130: // Set PID P Value
      if (GCode.HasWord('S')) { kp = GCode.GetWordValue('S'); }
      pid.SetTunings(kp, ki, kd);
      break;
    case 131: // Set PID I Value
      if (GCode.HasWord('S')) { ki = GCode.GetWordValue('S'); }
      pid.SetTunings(kp, ki, kd);
      break;
    case 132: // Set PID D Value
      if (GCode.HasWord('S')) { kd = GCode.GetWordValue('S'); }
      pid.SetTunings(kp, ki, kd);
      break;
    case 155: // Temperature Auto-Report
      if (GCode.HasWord('S')) { tempAutoReportInterval = (int)GCode.GetWordValue('S') * 1000; }
      break;
    case 301: // Set Hotend PID
      if (GCode.HasWord('P')) { kp = GCode.GetWordValue('P'); }
      if (GCode.HasWord('I')) { ki = GCode.GetWordValue('I'); }
      if (GCode.HasWord('D')) { kd = GCode.GetWordValue('D'); }
      pid.SetTunings(kp, ki, kd);
      break;
    // case 303: // Run PID tuning
    //   if (!GCode.HasWord('S')) { 
    //     Serial.print("Parameter S is required!\n");
    //   } else if (!GCode.HasWord('C')) { 
    //     Serial.print("Parameter C is required!\n");
    //   } else {
    //     // todo
    //     Serial.print("Kp: ");
    //     Serial.print(kp);
    //     Serial.print(" Ki: ");
    //     Serial.print(ki);
    //     Serial.print(" Kd: ");
    //     Serial.print(kd);
    //     Serial.print("\n");
    //     Serial.print("PID Autotune finished!\n");
    //   }
    //   break;
    case 500: // Store current settings to EEPROM
      saveToEEPROM();
      break;
    case 501: // Read all parameters from EEPROM
      loadFromEEPROM();
      break;
    case 502: // Restore current settings to defaults
      // https://insideautomation.net/initial-settings-pid-controllers/
      kp = 1.0;
      ki = 3.0;
      kd = 0.2;
      break;
    case 503: // Print the current settings
      Serial.print("kp = "); Serial.print(kp); Serial.print("\n");
      Serial.print("ki = "); Serial.print(ki); Serial.print("\n");
      Serial.print("kd = "); Serial.print(kd); Serial.print("\n");
      break;
    default:
      Serial.print("Unknown M command!\n");
      break;
  }
}

void heaterFault() {
  heaterPower = 0;
  targetTemp = 0;
  waitTemp = 0;
}

void loop() {
  uint32_t time = millis();

  computePid();
  // Serial.print(".");

  // thermal protection
  if (actualTemp1 < MINIMUM_VALUE || actualTemp1 > MAXIMUM_VALUE) {
    heaterFault();
    Serial.print("ERROR: Termister temperature out of bounds! (");
    Serial.print(MINIMUM_VALUE);
    Serial.print(" <= ");
    Serial.print(actualTemp1);
    Serial.print(" <= ");
    Serial.print(MAXIMUM_VALUE);
    Serial.print(")\n");
  }
  if (actualTemp2 < MINIMUM_VALUE || actualTemp2 > MAXIMUM_VALUE) {
    heaterFault();
    Serial.print("ERROR: Termister temperature out of bounds! (");
    Serial.print(MINIMUM_VALUE);
    Serial.print(" <= ");
    Serial.print(actualTemp2);
    Serial.print(" <= ");
    Serial.print(MAXIMUM_VALUE);
    Serial.print(")\n");
  }

  if ((time - tempCheckTime) > TEMP_CHECK_TIME * 1000) {
    if (tempCheckTime > 0) {
      float diffTemp = actualTemp - tempCheckTemp;
      if (actualTemp < targetTemp && diffTemp < TEMP_HEATING_GAIN) {
        heaterFault();
        Serial.print("ERROR: Heater not heating at expected rate!\n");
      }
    }
    tempCheckTemp = actualTemp;
    tempCheckTime = time;
  }


  if (tempAutoReportInterval > 0) {
    if (tempAutoReportTime == 0 || (time - tempAutoReportTime) > tempAutoReportInterval) {
      processCommandM(105);
      tempAutoReportTime = time;
    }
  }

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
