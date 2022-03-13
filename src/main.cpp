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

  // stolen from example
  Serial.print("Serial number: ");
  Serial.print(cam.serialNumber[0], HEX);
  Serial.print(cam.serialNumber[1], HEX);
  Serial.println(cam.serialNumber[2], HEX);

  // cam.setMode(MLX90640_INTERLEAVED);
  cam.setMode(MLX90640_CHESS);
  Serial.print("Current mode: ");
  if (cam.getMode() == MLX90640_CHESS)
  {
    Serial.println("Chess");
  }
  else
  {
    Serial.println("Interleave");
  }

  cam.setResolution(MLX90640_ADC_18BIT);
  Serial.print("Current resolution: ");
  mlx90640_resolution_t res = cam.getResolution();
  switch (res)
  {
  case MLX90640_ADC_16BIT:
    Serial.println("16 bit");
    break;
  case MLX90640_ADC_17BIT:
    Serial.println("17 bit");
    break;
  case MLX90640_ADC_18BIT:
    Serial.println("18 bit");
    break;
  case MLX90640_ADC_19BIT:
    Serial.println("19 bit");
    break;
  }

  cam.setRefreshRate(MLX90640_2_HZ);
  Serial.print("Current frame rate: ");
  mlx90640_refreshrate_t rate = cam.getRefreshRate();
  switch (rate)
  {
  case MLX90640_0_5_HZ:
    Serial.println("0.5 Hz");
    break;
  case MLX90640_1_HZ:
    Serial.println("1 Hz");
    break;
  case MLX90640_2_HZ:
    Serial.println("2 Hz");
    break;
  case MLX90640_4_HZ:
    Serial.println("4 Hz");
    break;
  case MLX90640_8_HZ:
    Serial.println("8 Hz");
    break;
  case MLX90640_16_HZ:
    Serial.println("16 Hz");
    break;
  case MLX90640_32_HZ:
    Serial.println("32 Hz");
    break;
  case MLX90640_64_HZ:
    Serial.println("64 Hz");
    break;
  }
}

void loop()
{
  // more stolen code
  delay(500);
  if (cam.getFrame(frame) != 0)
  {
    Serial.println("Failed");
    return;
  }
  Serial.println();
  Serial.println();
  for (uint8_t h = 0; h < 24; h++)
  {
    for (uint8_t w = 0; w < 32; w++)
    {
      float t = frame[h * 32 + w];
      Serial.print(t, 1);
      Serial.print(", ");
    }
  }
}