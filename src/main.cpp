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
#define STEPS_PER_DEGREE 0.556   // number of stepper motor steps per rotational degree
#define ANGLE 55                 // degrees between open and closed shutter position
#define LUT_NUM_VALS 16          // number of data points in the thermistor lookup table
#define STEPPER_MAX_SPEED 50     // maximum stepper speed in steps/sec
#define STEPPER_ACCELERATION 100 // stepper acceleration/deceleration rate in steps/sec^2

// ~~~~~ Global Variables ~~~~~
AccelStepper stepper = AccelStepper(AccelStepper::FULL4WIRE, A1, A2, B1, B2);
Adafruit_MLX90640 cam;
Adafruit_DotStar led(1, LED_DATA, LED_CLK, DOTSTAR_BRG);

float frame[32 * 24]; // buffer for full frame of temperatures

const int BUFFER_LENGTH = 3; // number of characters to read from the serial buffer, 3 characters to contain CMD_START, the command byte, and CMD_END

const byte PING_REQUEST = 'p';    // prompts a 'pong' response
const byte PING_RESPONSE = 'o';   // byte to send as 'pong' response
const byte FRAME_REQUEST = 'r';   // prompts a regular image to be taken and sent
const byte HOME_REQUEST = 'h';    // prompts device to home stepper motor
const byte CAL_REQUEST = 'c';     // prompts device to calibrate image temperature values
const byte SHUTTER_REQUEST = 's'; // prompts an image of the closed shutter to be taken and transmitted
const byte DF_START = '[';        // demarcates the beginning of a data frame
const byte DF_END = ']';          // demarcates the end of a data frame
const byte CMD_START = '<';       // demarcates the beginning of a serial command
const byte CMD_END = '>';         // demarcates the end of a serial command
const byte LINE_END = '\n';       // byte to signify the end of a serial line

// lookup table of temperatures in deg C and respective thermistor resistances in ohms
const int lookupTable[LUT_NUM_VALS][2] = {{0,321140},{20, 124692}, {21, 119253}, {22, 114078}, {23, 109152}, {24, 104464}, {25, 100000}, {26, 95747}, {27, 91697}, {28, 87837}, {29, 84157}, {30, 80650}, {31, 77305}, {32, 74115}, {33, 71072},{60,24681}};
const float R3 = 100100.0; // Resistance of thermistor voltage divider resistor R3

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

// Returns the temperature of the thermistor in degrees C
float readThermistor()
{
  int raw = analogRead(TEMP);
  float V = raw * 3.3 / 4096.0;
  float R = 3.3 * (R3 / V) - R3;
  return resistanceToTemp(R);
}

// Returns the state of the optical proximity sensor, true if obstructed, else false
bool opticalSensor()
{
  digitalWrite(OPTO_EN, HIGH);
  delay(10);
  bool tmp = digitalRead(OPTO);
  digitalWrite(OPTO_EN, LOW);
  return !tmp;
}

// Moves the stepper motor some number of steps
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

// Rotates the calibration shutter counterclockwise until optical sensor is triggered or timeout is reached, returns success or failure
bool homeStepper()
{
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
    if ((int)millis() > timeout)
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

// Returns the average uncorrected temperature value of all pixels in the sensor
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

// Overwrites the calibration of the camera temperature
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

// Take a single exposure and send it over serial
void sendImage()
{
  cam.getFrame(frame);
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
}

// Send an image of the closed shutter over serial
void sendShutterImage()
{
  // ensure shutter is positioned over lens
  if (opticalSensor())
  {
    homeStepper(); // locate shutter and position it over lens
    delay(1000);   // allow image to settle
  }
  sendImage();
  delay(1000);
}

// Open the shutter, take a single frame, and send over serial
void sendFrame()
{
  moveStepper(-1 * (int)(ANGLE * STEPS_PER_DEGREE));
  delay(1000);
  sendImage();
  delay(1000);
  moveStepper((int)(ANGLE * STEPS_PER_DEGREE));
}

// Send a response to a ping over serial
void sendPong()
{
  Serial.write(CMD_START);
  Serial.write(PING_RESPONSE);
  Serial.write(CMD_END);
  Serial.write(LINE_END);
}

// monitor incoming serial communications and respond accordingly
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

      case SHUTTER_REQUEST:
        sendShutterImage();
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

// ~~~~~~~~~~ Main Functions ~~~~~~~~~~~~~

void setup()
{
  while (!Serial)
  {
    delay(10); // If serial connection not active, wait doing nothing
  }
  Serial.begin(115200);
  Serial.println("~~~~~~ SERIAL INITIATED ~~~~~~");

  pinMode(TEMP, INPUT_PULLDOWN);
  pinMode(OPTO, INPUT_PULLDOWN);
  pinMode(OPTO_EN, OUTPUT);
  pinMode(SLP, OUTPUT);
  digitalWrite(OPTO_EN, LOW);
  digitalWrite(SLP, LOW);

  analogReadResolution(12); // Set analog read resolution to 12 bits (4096 vals)
  led.begin();
  led.show();

  if (!cam.begin(MLX90640_I2CADDR_DEFAULT, &Wire))
  {
    while (true)
    {
      Serial.println("!!!!!MLX90640 not found!!!!!");
      delay(1000);
    }
  }
  else
  {
    Serial.println("MLX90640 detected. Configuring.");
    cam.setMode(MLX90640_INTERLEAVED);
    // cam.setMode(MLX90640_CHESS);
    cam.setResolution(MLX90640_ADC_18BIT);
    cam.setRefreshRate(MLX90640_2_HZ);
  }

  stepper.setMaxSpeed(STEPPER_MAX_SPEED);
  stepper.setAcceleration(STEPPER_ACCELERATION);

  if (!homeStepper())
  {
    while (true)
    {
      Serial.println("!!!!!Stepper homing failed!!!!!!");
      delay(1000);
    }
  }
  else
  {
    Serial.println("Stepper homing successful.");
  }
}

void loop()
{
  watchSerial();
}