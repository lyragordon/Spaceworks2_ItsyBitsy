#include <Arduino.h>
#include <AccelStepper.h>
#include <Adafruit_MLX90640.h>
#include <Adafruit_BusIO_Register.h>

// ~~~~~ Pin Definitions ~~~~~
// Stepper motor
#define A1 9
#define A2 10
#define B1 13
#define B2 12
#define SLP 11
// I/O
#define OPTO_EN 7
#define OPTO 5
#define TEMP A4

// ~~~~~ Global Variables ~~~~~

int inputPins[] = {TEMP, OPTO};
int outputPins[] = {OPTO_EN, SLP};

AccelStepper stepper = AccelStepper(AccelStepper::FULL4WIRE, A1, A2, B1, B2);
Adafruit_MLX90640 cam;

float frame[32 * 24]; // buffer for full frame of temperatures

const int BUFFER_LENGTH = 3;

const byte PING_REQUEST = 'p';
const byte PING_RESPONSE = 'o';
const byte FRAME_REQUEST = 'r';
const byte DF_START = '[';
const byte DF_END = ']';
const byte CMD_START = '<';
const byte CMD_END = '>';
const byte LINE_END = '\n';

// ~~~~~ Support Functions ~~~~~

bool move(int relative)
{
  stepper.move(relative);
  while (stepper.distanceToGo())
  {
    stepper.run();
  }
}

bool opticalSensor()
{
  digitalWrite(OPTO_EN, HIGH);
  bool tmp = digitalRead(OPTO);
  digitalWrite(OPTO_EN, LOW);
  return tmp;
}

bool homeStepper()
{
  // Rotates the calibration shutter counterclockwise until optical sensor is triggered or timeout is reached, returns success or failure
  int timeout = millis() + 10000; // 10 second timeout
  while (opticalSensor() == false)
  {
    move(-1); // Rotate 1 step CCW
    if (millis() > timeout)
    {
      return false;
    }
  }
  stepper.setCurrentPosition(0); // Set open shutter to position 0
  return true;
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
    pinMode(inputPins[i], INPUT);
  }
  for (int i = 0; i < 2; i++)
  {
    pinMode(outputPins[i], OUTPUT);
  }

  if (!cam.begin(MLX90640_I2CADDR_DEFAULT, &Wire))
  {
    Serial.println("!!!!!MLX90640 not found!!!!!");
  }
  cam.setMode(MLX90640_INTERLEAVED);
  // cam.setMode(MLX90640_CHESS);
  cam.setResolution(MLX90640_ADC_18BIT);
  cam.setRefreshRate(MLX90640_2_HZ);

  stepper.setMaxSpeed(200); // max speed of 1 rotation per second
  // stepper.setAcceleration()
  if (!homeStepper())
  {
    Serial.println("!!!!!Stepper homing failed!!!!!!");
  }
}

void sendFrame()
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