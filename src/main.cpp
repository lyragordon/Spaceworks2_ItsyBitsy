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

int BUFFER_LENGTH = 1;

byte PING_REQUEST = 'p';
byte PING_RESPONSE = 'o';
byte FRAME_REQUEST = 'r';
byte DF_START = '[';
byte DF_END = ']';
byte CMD_END = '?';
byte RESPONSE_END = '\n';

// ~~~~~ Support Functions ~~~~~

bool homeStepper()
{
  // Rotates the calibration shutter counterclockwise until optical sensor is triggered or timeout is reached, returns success or failure
  return false;
}

void setup()
{
  while (!Serial)
  {
    delay(10);
  }
  Serial.begin(115200);
  Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
  Serial.println("~~~~~~ SERIAL INITIATED ~~~~~~");
  Serial.println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");

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

  if (!homeStepper())
  {
    Serial.println("!!!!!Stepper homing failed!!!!!!");
  }

  // cam.setMode(MLX90640_INTERLEAVED);
  cam.setMode(MLX90640_CHESS);

  cam.setResolution(MLX90640_ADC_18BIT);

  cam.setRefreshRate(MLX90640_2_HZ);
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
  Serial.write(RESPONSE_END);
}

void loop()
{
  byte buffer[BUFFER_LENGTH];
  if (Serial.available())
  {
    Serial.readBytesUntil('?', buffer, BUFFER_LENGTH);
    if (buffer[0] == PING_REQUEST)
    {
      Serial.write(PING_RESPONSE);
      Serial.write(RESPONSE_END);
    }
    else if (buffer[0] == FRAME_REQUEST)
    {
      sendFrame();
    }
  }
}