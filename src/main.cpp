#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

// Angular positions for the servo
#define START_ANGLE 90
#define END_ANGLE 120


// Declare global objects
MPU6050 mpu;
Servo servo;

// Declare global variables
boolean ledState = false;
boolean freefallDetected = false;
int freefallBlinkCount = 0;
int servo_pos = 0;
int servo_flag = 1;

// ISR that triggers when freefall is detected
void ff_isr()
{
  freefallBlinkCount = 0;
  freefallDetected = true;  
}

// Function to check and display the current settings of the MPU6050 sensor
void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:                ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Motion Interrupt:     ");
  Serial.println(mpu.getIntMotionEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Zero Motion Interrupt:     ");
  Serial.println(mpu.getIntZeroMotionEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Free Fall Interrupt:       ");
  Serial.println(mpu.getIntFreeFallEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Free Fal Threshold:          ");
  Serial.println(mpu.getFreeFallDetectionThreshold());

  Serial.print(" * Free FallDuration:           ");
  Serial.println(mpu.getFreeFallDetectionDuration());
  
  Serial.print(" * Clock Source:              ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:             ");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Accelerometer offsets:     ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());

  Serial.print(" * Accelerometer power delay: ");
  switch(mpu.getAccelPowerOnDelay())
  {
    case MPU6050_DELAY_3MS:            Serial.println("3ms"); break;
    case MPU6050_DELAY_2MS:            Serial.println("2ms"); break;
    case MPU6050_DELAY_1MS:            Serial.println("1ms"); break;
    case MPU6050_NO_DELAY:             Serial.println("0ms"); break;
  }  
  
  Serial.println();
}

void setup() 
{
  Serial.begin(115200);

  Serial.println("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  mpu.setAccelPowerOnDelay(MPU6050_DELAY_3MS);
  
  // Enable free-fall mode
  mpu.setIntFreeFallEnabled(true);
  mpu.setIntZeroMotionEnabled(false);
  mpu.setIntMotionEnabled(false);
  
  // Digital high pass filter
  mpu.setDHPFMode(MPU6050_DHPF_5HZ);

  // Set FF_THRES and FF_DUR 
  mpu.setFreeFallDetectionThreshold(120);
  mpu.setFreeFallDetectionDuration(1);	
  
  // Display current IMU settings
  checkSettings();

  // Attach the intterupt to an Arduino interrupt pin
  attachInterrupt(digitalPinToInterrupt(2), ff_isr, RISING);

  // Attach the servo to pin 9
  servo.attach(9);

}

void loop()
{
  // Checks for the freefall state 
  Activites act = mpu.readActivites();

  Serial.print(String(act.isFreeFall));

  // This block is run when the ff_isr() is triggered
  if (freefallDetected)
  {
    // Toggle LED state
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);

  // Actuate the servo only once
  if (servo_flag)
  {
    for (servo_pos = START_ANGLE; servo_pos <= END_ANGLE; servo_pos += 1) { 
      // in steps of 1 degree
      servo.write(servo_pos);              
      delay(15);                       
    }

    // Set flag to 0 so that it can never run again
    servo_flag = 0;
  }


    freefallBlinkCount++;

    // Stops blinking the LED after 20 blinks and resets the freefall state
    if (freefallBlinkCount == 20)
    {
      freefallDetected = false;
      ledState = false;
      digitalWrite(LED_BUILTIN, ledState);
    }
  }
  
  delay(100);
}


