// The idea here is to use Rejsa's code / PCB to implement a CAN / 12v powered version
// outputting in the Alsense DBC (https://www.alsense.eu/wp-content/uploads/2021/01/als_tire_wired-1.dbc) format
// Distance currently NOT IMPLEMENTED
//
// Cut the CAN termination trace on the top of the board next to the CAN pins
//
// Install Arduino SAMD support in arduino board manager
// Install Adafruit SAMD support in arduino board manager
// Install Adafruit CAN library
// Install VL53L0X library
// Install Adafruit NeoPixel library

#include <CANSAME5x.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include "MLX90621.h"
#include <SPI.h>
#include <Wire.h>
#include <VL53L0X.h>

CANSAME5x can;
VL53L0X distSensor;
MLX90621 tempSensor;
Adafruit_NeoPixel strip(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Defines the wheel position of this specific module, if set to 7 then it will be based off of the GPIO pins.
// If no jumpers are installed and WHEELPOS is set to 7 then no data will be transmitted
// 0 => FL
// 1 => FR
// 2 => RL
// 3 => RR
// 5 => F
// 6 => R
// 7 => Read from GPIO
#define WHEELPOS 7

// Defines the direction of the tire temperatures, eg. inside to outside vs outside to inside
// This can be overriden with a GPIO pin
// 0 => default
// 1 => Mirror the tire, making the outside edge temps the inside edge temps (eg. right side of car)
#define MIRRORTIRE 0
                          
// Used to define the base sensor distance from the tire surface so that the ranging sensor reports relative distance
// Set this as the distance to the tire in mm
#define DISTANCEOFFSET 0

// Enable transmission of fake random data for testing with no sensors needed
#ifdef ENABLE_DUMMY_TRANSMIT
  #define DUMMYDATA
#endif

// ----------------------------------------

// Default = 1.00
#define TEMPSCALING 1.00

// Default = 0 NOTE: in TENTHS of degrees Celsius --> TEMPOFFSET 10 --> 1 degree
#define TEMPOFFSET 0

// GPIO pin number, for distance sensor
#define GPIODISTSENSORXSHUT 25

// GPIO pin number, represents if this is a car or a bike (4 wheels vs 2 wheels)
#define GPIOCAR 18

// GPIO pin number, for setting front / rear
#define GPIOFRONT 19

// GPIO pin number, for setting left / right
#define GPIOLEFT 24

// GPIO pin number, for setting mirrored temperature reporting
#define GPIOMIRR 23

// Number between 20 and 100 (milliseconds)
// Lower number can give a higher update rate of all data over Bluetooth (especially if only 8 temperatures are fetched)
// Higher number reduces noise on the distance measurements considerably
#define DISTMEASUREBUDGET 80

uint8_t distSensorPresent = 0;
uint8_t tempSensorPresent = 0;
uint8_t mirrorTire = 0;
uint8_t wheelPositionCode;
uint32_t lastSendTimestamp = 0;

// ----------------------------------------
// Function declarations

uint8_t initDistanceSensor(void);
int16_t distanceFilter(int16_t);
uint8_t getWheelPositionCoding(void);
uint16_t getBaseCanIdForWheel(uint8_t);
void printStatus(void);

typedef struct {
  // Millimeters
  int16_t distance;
  // Temperature in degrees celsius x 10
  int16_t temperatures[16];
} structuredData;

structuredData dataPackage;

// ----------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  Serial.println("\nBegin startup");

  // GPIO pin setup
  pinMode(GPIODISTSENSORXSHUT, OUTPUT);
  pinMode(GPIOLEFT, INPUT_PULLUP);
  pinMode(GPIOFRONT, INPUT_PULLUP);
  pinMode(GPIOCAR, INPUT_PULLUP);
  pinMode(GPIOMIRR, INPUT_PULLUP);

  // CAN pin setup
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false);
  // This enables 5v for CAN
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true);

  Serial.println("Starting I2C");
  Wire.begin();

  // Distance sensor
  Serial.println("Starting distance sensor");
  distSensorPresent = initDistanceSensor();
  Serial.print("Distance sensor is ");
  Serial.println(distSensorPresent ? "present" : "not present");

  // Temperature sensor
  Serial.println("Starting temperature sensor in 16 pixel mode");
  Wire.beginTransmission(0x60);
  if (Wire.endTransmission() != 0) {
    Serial.println("Temperature sensor not detected");
  } else {
    tempSensor.initialise(16);
    tempSensorPresent = 1;
  }

  // Wheel position
  wheelPositionCode = getWheelPositionCoding();
  Serial.print("Wheel position set to ");
  Serial.println(getWheelPositionString(wheelPositionCode));

  // Mirroring of sensor data
  mirrorTire = (MIRRORTIRE == 1 || digitalRead(GPIOMIRR) == 0) ? 1 : 0;
  Serial.print("Temperature sensor ");
  Serial.println(mirrorTire ? "is mirrored" : "is not mirrored");

  // CANBUS
  Serial.println("Initializing CAN bus...");
  if (!can.begin(1000000)) {
    Serial.println("Failed to initialize CAN!");
    while (1);
  }
  Serial.println("CAN initialized at 1000kbps");

  // Neopixel because why not
  strip.begin();
  strip.setBrightness(5);
  strip.show();
  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();

  Serial.println("Startup complete");
}

// ----------------------------------------

void loop() {
  // Limit to 5hz, don't need that fast but whatever
  if (millis() - lastSendTimestamp < 200) return;
  lastSendTimestamp = millis();

  // Distance
  if (distSensorPresent) {
    uint16_t distance = distSensor.readRangeContinuousMillimeters();

    if (distance == 8190) {
      // Nothing detected
      dataPackage.distance = 0;
    } else {
      dataPackage.distance = distanceFilter(distance) - DISTANCEOFFSET;
    }
  }

  // Temperature
  if (tempSensorPresent) {
    tempSensor.measure(true);

    for (uint8_t i = 0; i < 8; i++) {
      uint8_t idx = i;

      if (mirrorTire == 1) {
        idx = 7 - i;
      }
    
      int16_t maxEven = (int16_t)(TEMPOFFSET + TEMPSCALING * 10 * max(tempSensor.getTemperature(i * 8 + 1), tempSensor.getTemperature(i * 8 + 2)));
      int16_t maxOdd = (int16_t)(TEMPOFFSET + TEMPSCALING * 10 * max(tempSensor.getTemperature(i * 8 + 5), tempSensor.getTemperature(i * 8 + 6)));
    
      dataPackage.temperatures[idx * 2] = maxEven;
      dataPackage.temperatures[idx * 2 + 1] = maxOdd;
    }
  }
  
  // CANBUS
  uint16_t baseId = getBaseCanIdForWheel(wheelPositionCode);
  if (baseId == 0) {
    Serial.println("Invalid wheel position detected");
    return;
  }

  // Send temps in chunks of 4 (Chan 0 to Chan 3)
  for (uint8_t frame = 0; frame < 4; frame++) {
    uint8_t payload[8];

    for (uint8_t i = 0; i < 4; i++) {
      uint8_t tempIndex = frame * 4 + i;
      int16_t rawTemp = dataPackage.temperatures[tempIndex];
      int16_t adjusted = rawTemp + 2000; // offset of 200c to match DBC

      payload[i * 2 + 0] = (adjusted >> 8) & 0xFF; // MSB
      payload[i * 2 + 1] = adjusted & 0xFF; // LSB
    }

    can.beginPacket(baseId + 0x10 + frame);
    can.write(payload, 8);
    can.endPacket();
  }

  // TODO add a DATA_SUMMARY message that also includes distance and whatever else
  // can.beginPacket(baseId + 0x14);
  Serial.println("Tick completed");
}

// Initialize the distance sensor
uint8_t initDistanceSensor(void) {
  digitalWrite(GPIODISTSENSORXSHUT, LOW);
  delay(50);

  digitalWrite(GPIODISTSENSORXSHUT, HIGH);
  delay(50);

  distSensor.setTimeout(150);

  if (!distSensor.init()) {
    return 0; 
  }

  distSensor.setMeasurementTimingBudget(DISTMEASUREBUDGET * 1000);
  distSensor.startContinuous();

  return 1; 
}

// Get a distance after doing some moving average filtering
int16_t distanceFilter(int16_t distanceIn) {
  const uint8_t filterSz = 2;
  static int16_t filterArr[filterSz];
  int16_t distanceOut = 0;

  for (int8_t i = 0; i < (filterSz - 1); i++) {
    filterArr[i] = filterArr[i + 1];
    distanceOut += filterArr[i + 1];
  }

  filterArr[filterSz - 1] = distanceIn;
  distanceOut += distanceIn;
  
  return (int16_t) distanceOut / filterSz;
}

// Gets the wheel position based on the GPIO pins or hard coding
uint8_t getWheelPositionCoding(void) {
  return (WHEELPOS < 7) ? WHEELPOS : digitalRead(GPIOLEFT) + (digitalRead(GPIOFRONT) << 1) + (digitalRead(GPIOCAR) << 2);
}

// Gets a string representation of the set wheel position
const char *getWheelPositionString(uint8_t code) {
  switch (code) {
      case 0: return "FL";
      case 1: return "FR";
      case 2: return "RL";
      case 3: return "RR";
      case 5: return "F";
      case 6: return "R";
      default: return "Unknown";
  }
}

// Gets a base CAN ID for the set wheel position
uint16_t getBaseCanIdForWheel(uint8_t code) {
  switch (code) {
    case 0: return 784; // FL
    case 1: return 816; // FR
    case 2: return 848; // RL
    case 3: return 880; // RR
    case 5: return 784; // F
    case 6: return 880; // R
    default: return 0;  // Unknown
  }
}