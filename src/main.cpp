#include <Arduino.h>
#include <AccelStepper.h>
#include <Adafruit_MLX90640.h>

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

// ~~~~~ Support Functions ~~~~~

int len(int a[])
{
  // This will get compiled away so it's not a performance hit, just convenient
  return sizeof(a) / sizeof(a[0]);
}

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

  for (int i = 0; i < len(inputPins); i++)
  {
    pinMode(inputPins[i], INPUT);
  }
  for (int i = 0; i < len(outputPins); i++)
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
}

void loop()
{
  // put your main code here, to run repeatedly:
}