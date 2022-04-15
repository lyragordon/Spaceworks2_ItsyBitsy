#include <Arduino.h>
#include <AccelStepper.h>
#include <Adafruit_MLX90640.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_DotStar.h>

// ~~~~~ Pin Definitions ~~~~~
// Stepper motor
#define A1 9
#define A2 10
#define B1 13
#define B2 12
#define SLP 11
// I/O
#define OPTO_EN 7
#define OPTO 1
#define TEMP A4
#define LED_CLK 6
#define LED_DATA 8

// ~~~~~ Macros ~~~~~
#define STEPS_PER_DEGREE 0.556
#define ANGLE 55 // degrees between open and closed shutter position

// ~~~~~ Global Variables ~~~~~

int inputPins[] = {TEMP, OPTO};
int outputPins[] = {OPTO_EN, SLP};

AccelStepper stepper = AccelStepper(AccelStepper::FULL4WIRE, A1, A2, B1, B2);
Adafruit_MLX90640 cam;
Adafruit_DotStar led(1, LED_DATA, LED_CLK, DOTSTAR_BRG);

float frame[32 * 24]; // buffer for full frame of temperatures

const int BUFFER_LENGTH = 3;

const byte PING_REQUEST = 'p';
const byte PING_RESPONSE = 'o';
const byte FRAME_REQUEST = 'r';
const byte HOME_REQUEST = 'h';
const byte CAL_REQUEST = 'c';
const byte DF_START = '[';
const byte DF_END = ']';
const byte CMD_START = '<';
const byte CMD_END = '>';
const byte LINE_END = '\n';

#define LUT_NUM_VALS 11
const int lookupTable[LUT_NUM_VALS][2] = {{20, 124692}, {21, 119253}, {22, 114078}, {23, 109152}, {24, 104464}, {25, 100000}, {26, 95747}, {27, 91697}, {28, 87837}, {29, 84157}, {30, 80650}};
const float R3 = 100100.0; // Resistance of thermistor voltage divider resistor

// ~~~~~ Support Functions ~~~~~

float resistanceToTemp(float R)
{
  for (int i = 0; i < (LUT_NUM_VALS - 1); i++)
  {
    if (R < lookupTable[i][1] && R > lookupTable[i + 1][1])
    {
      // linear interpolation
      return (lookupTable[i + 1][0] - lookupTable[i][0]) * (R - lookupTable[i][1]) / (lookupTable[i + 1][1] - lookupTable[i][1]) + lookupTable[i][0];
    }
    else if (R == lookupTable[i][1])
    {
      return (float)lookupTable[i][0];
    }
    else if (R == lookupTable[i + 1][1])
    {
      return (float)lookupTable[i + 1][0];
    }
  }
  // if resistance greater than max in LUT or less than min in LUT,
  return -1.0;
}

float readThermistor()
{
  int raw = analogRead(TEMP);
  float V = raw * 3.3 / 4096.0;
  float R = 3.3 * (R3 / V) - R3;
  return resistanceToTemp(R);
}

bool opticalSensor()
{
  digitalWrite(OPTO_EN, HIGH);
  delay(10);
  bool tmp = digitalRead(OPTO);
  digitalWrite(OPTO_EN, LOW);
  return !tmp;
}

void moveStepper(int steps)
{
  digitalWrite(SLP, HIGH);
  stepper.move(steps);
  while (stepper.distanceToGo())
  {
    stepper.run();
  }
  digitalWrite(SLP, LOW);
}

bool homeStepper()
{
  // Rotates the calibration shutter counterclockwise until optical sensor is triggered or timeout is reached, returns success or failure
  if (opticalSensor())
  {
    moveStepper(30);
  }
  int timeout = millis() + 20000; // 20 second timeout
  digitalWrite(SLP, HIGH);
  stepper.move(-1000);
  while (opticalSensor() == false)
  {
    stepper.run();
    if (millis() > timeout)
    {
      digitalWrite(SLP, LOW);
      return false;
    }
  }
  stepper.setCurrentPosition(0); // Set open shutter to position 0
  stepper.move((int)(ANGLE * STEPS_PER_DEGREE));
  while (stepper.distanceToGo())
  {
    stepper.run();
  }
  digitalWrite(SLP, LOW);
  return true;
}

float cameraReadAvgTemp()
{
  cam.getFrame(frame); // load camera data into frame buffer
  float avg = 0;
  for (int i = 0; i < 32 * 24; i++)
  {
    avg += frame[i];
  }
  avg = avg / (32 * 24);
  return avg;
}

void calTemp()
{
  if (opticalSensor())
  {
    homeStepper();
    delay(1000); // let reading settle
  }

  float thermistorReading = readThermistor();
  float cameraReading = cameraReadAvgTemp();
  Serial.print("thermistor: ");
  Serial.print(thermistorReading);
  Serial.print(" | cam avg: ");
  Serial.println(cameraReading);
}

void setup()
{
  while (!Serial)
  {
    delay(10);
  }
  Serial.begin(115200);
  Serial.println("~~~~~~ SERIAL INITIATED ~~~~~~");

  for (int i = 0; i < 2; i++)
  {
    pinMode(inputPins[i], INPUT_PULLDOWN);
  }
  for (int i = 0; i < 2; i++)
  {
    pinMode(outputPins[i], OUTPUT);
    digitalWrite(outputPins[i], LOW);
  }
  analogReadResolution(12); // Set analog read resolution to 12 bits (4096 vals)
  led.begin();
  led.show();

  if (!cam.begin(MLX90640_I2CADDR_DEFAULT, &Wire))
  {
    Serial.println("!!!!!MLX90640 not found!!!!!");
    // while (true)
    // {
    //   ; // hang
    // }
  }
  else
  {
    Serial.println("MLX90640 detected. Configuring.");
    cam.setMode(MLX90640_INTERLEAVED);
    // cam.setMode(MLX90640_CHESS);
    cam.setResolution(MLX90640_ADC_18BIT);
    cam.setRefreshRate(MLX90640_2_HZ);
  }

  stepper.setMaxSpeed(50.0);
  stepper.setAcceleration(100.0);

  if (!homeStepper())
  {
    Serial.println("!!!!!Stepper homing failed!!!!!!");
    while (true)
    {
      ; // hang
    }
  }
  else
  {
    Serial.println("Stepper homing successful.");
  }
}

void sendFrame()
{
  moveStepper(-1 * (int)(ANGLE * STEPS_PER_DEGREE));
  delay(1000);
  cam.getFrame(frame);
  delay(1000);
  Serial.write(DF_START);
  for (uint8_t h = 0; h < 24; h++)
  {
    for (uint8_t w = 0; w < 32; w++)
    {
      float t = frame[h * 32 + w];
      Serial.print(t, 1);
      if (h * 32 + w != 767)
      {
        Serial.print(',');
      }
    }
  }
  Serial.write(DF_END);
  Serial.write(LINE_END);
  moveStepper((int)(ANGLE * STEPS_PER_DEGREE));
}

void sendPong()
{
  Serial.write(CMD_START);
  Serial.write(PING_RESPONSE);
  Serial.write(CMD_END);
  Serial.write(LINE_END);
}

void watchSerial()
{
  if (Serial.available())
  {
    char buffer[BUFFER_LENGTH];
    Serial.readBytes(buffer, BUFFER_LENGTH);

    if (buffer[0] == CMD_START && buffer[2] == CMD_END)
    {
      switch (buffer[1])
      {
      case PING_REQUEST:
        sendPong();
        break;

      case FRAME_REQUEST:
        sendFrame();
        break;

      case HOME_REQUEST:
        homeStepper();
        break;

      case CAL_REQUEST:
        calTemp();
        break;

      default:
        Serial.print("Invalid serial command '");
        Serial.write(buffer[1]);
        Serial.println("'.");
        break;
      }
    }
  }
}

void loop()
{
  watchSerial();
}