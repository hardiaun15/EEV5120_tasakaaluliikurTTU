
#Finclude <Wire.h>

// ==========================================
// 1. MPU6050 SEADISTUS
// ==========================================

#define MPU_ADDR 0x68

float gyroY = 0;
float gyroOffsetY = 0;

float accAngle = 0;
float robotAngle = 0;

// Complementary filter
float alpha = 0.98;

// ==========================================
// 2. MOOTORITE VIIGUD
// ==========================================

// Vasak mootor - kanal A
#define PWMA 9
#define AIN1 8
#define AIN2 7

// Parem mootor - kanal B
#define PWMB 10
#define BIN1 12
#define BIN2 11

#define STBY 6

// Muuda vastavalt mootorite juhtmete uhendusele
#define LEFT_MOTOR_REVERSED true
#define RIGHT_MOTOR_REVERSED false

// ==========================================
// 3. ENKOODERID
// ==========================================

#define ENC1_A 2
#define ENC1_B 4

#define ENC2_A 3
#define ENC2_B 5

volatile long encoderLeft = 0;
volatile long encoderRight = 0;

void encoderLeftISR()
{
  if (digitalRead(ENC1_B))
    encoderLeft++;
  else
    encoderLeft--;
}

void encoderRightISR()
{
  if (digitalRead(ENC2_B))
    encoderRight++;
  else
    encoderRight--;
}

// ==========================================
// 4. AKU PINGE MooTMINE
// ==========================================

#define BATTERY_PIN A0

float batteryVoltage = 0;

const float BATTERY_MIN = 6.4;
const float ADC_REFERENCE = 5.0;
const float DIVIDER_RATIO = 2.0;

// ==========================================
// 5. PID REGULAATOR
// ==========================================

// Algvaartused - vajavad reaalsel robotil haalestamist

float Kp = 22.0;
float Ki = 0.0;
float Kd = 0.9;

float targetAngle = 0.0;

float pidError = 0;
float integral = 0;
float pidOutput = 0;

float maxOutput = 255;

// ==========================================
// 6. AJASTUS
// ==========================================

unsigned long previousTime = 0;

const unsigned long LOOP_TIME = 5000;
// 5000 mikrosekundit = 5 ms
// 200 arvutust sekundis

// ==========================================
// 7. MPU6050 REGISTRI KIRJUTAMINE
// ==========================================

void writeMPU(byte reg, byte data)
{

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// ==========================================
// 8. MPU6050 ALGSEADISTUS
// ==========================================

void setupMPU()
{

  Wire.begin();

  // MPU6050 aratamine
  writeMPU(0x6B, 0x00);

  // Guroskoop +/- 250 kraadi/s
  writeMPU(0x1B, 0x00);

  // Kiirendusandur +/- 2g
  writeMPU(0x1C, 0x00);

  delay(100);
}

// ==========================================
// 9. MPU6050 ANDMETE LUGEMINE
// ==========================================

void readMPU()
{

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 14, true);

  if (Wire.available() < 14)
  {
    return;
  }

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();

  Wire.read();
  Wire.read();

  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();

  // Kiirendusanduri kaldenurk
  // Eeldab, et roboti kalde telg on Y

  accAngle = atan2((float)ax, (float)az) * 180.0 / PI;

  // Güroskoobi nurkkiirus kraadides sekundis
  gyroY = (float)gy / 131.0;

  // Kalibreerimisvea eemaldamine
  gyroY -= gyroOffsetY;
}

// ==========================================
// 10. GuROSKOOBI KALIBREERIMINE
// ==========================================

void calibrateGyro()
{

  long sum = 0;

  Serial.println("Gyro calibration...");
  Serial.println("Keep robot still!");

  for (int i = 0; i < 1000; i++)
  {

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x45);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU_ADDR, 2, true);

    if (Wire.available() == 2)
    {

      int16_t gy = Wire.read() << 8 | Wire.read();

      sum += gy;
    }

    delay(3);
  }

  gyroOffsetY = (float)sum / 1000.0 / 131.0;

  Serial.println("Calibration finished");
}

// ==========================================
// 11. PID ARVUTAMINE
// ==========================================

void calculatePID(float dt)
{

  pidError = robotAngle - targetAngle;

  integral += pidError * dt;

  // Integraali piiramine
  integral = constrain(integral, -30, 30);

  // Positiivne väljund peab liigutama rattaid
  // roboti kukkumise suunas.
  pidOutput =
      Kp * pidError + Ki * integral - Kd * gyroY;

  pidOutput = constrain(
      pidOutput,
      -maxOutput,
      maxOutput);
}

// ==========================================
// 12. MOOTORI JUHTIMINE
// ==========================================

void setMotor(int pwm, bool channelA, bool reversed)
{

  pwm = constrain(pwm, -255, 255);

  if (reversed)
  {
    pwm = -pwm;
  }

  int speed = abs(pwm);

  if (channelA)
  {

    if (pwm > 0)
    {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    }
    else if (pwm < 0)
    {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
    }
    else
    {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
    }

    analogWrite(PWMA, speed);
  }

  else
  {

    if (pwm > 0)
    {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    }
    else if (pwm < 0)
    {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    }
    else
    {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);
    }

    analogWrite(PWMB, speed);
  }
}

// ==========================================
// 13. KAHE MOOTORI JUHTIMINE
// ==========================================

void driveMotors(int output)
{

  setMotor(
      output,
      true,
      LEFT_MOTOR_REVERSED);

  setMotor(
      output,
      false,
      RIGHT_MOTOR_REVERSED);
}

// ==========================================
// 14. MOOTORITE SEISKAMINE
// ==========================================

void stopMotors()
{

  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  digitalWrite(STBY, LOW);
}

// ==========================================
// 15. AKU PINGE MooTMINE
// ==========================================

void readBattery()
{

  int adc = analogRead(BATTERY_PIN);

  batteryVoltage =
      adc * ADC_REFERENCE / 1023.0 * DIVIDER_RATIO;
}

// ==========================================
// 16. ARDUINO SETUP
// ==========================================

void setup()
{

  Serial.begin(115200);

  // Mootorite viigud
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  // Enkooderid
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);

  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  attachInterrupt(
      digitalPinToInterrupt(ENC1_A),
      encoderLeftISR,
      RISING);

  attachInterrupt(
      digitalPinToInterrupt(ENC2_A),
      encoderRightISR,
      RISING);

  stopMotors();

  // IMU kaivitamine
  setupMPU();

  // Guroskoobi kalibreerimine
  calibrateGyro();

  // Esimese kaldenurga lugemine
  readMPU();

  robotAngle = accAngle;

  previousTime = micros();

  Serial.println("Balancing robot ready");
}

// ==========================================
// 17. PoHIPROGRAMM
// ==========================================

void loop()
{

  unsigned long currentTime = micros();

  if (currentTime - previousTime >= LOOP_TIME)
  {

    float dt =
        (currentTime - previousTime) / 1000000.0;

    previousTime = currentTime;

    // IMU andmete lugemine
    readMPU();

    // Complementary filter
    robotAngle =
        alpha * (robotAngle + gyroY * dt) + (1.0 - alpha) * accAngle;

    // Aku pinge mootmine
    readBattery();

    // Ohutus: aku liiga tuhi
    if (batteryVoltage < BATTERY_MIN)
    {

      stopMotors();

      Serial.println("LOW BATTERY!");

      return;
    }

    // Ohutus: robot liiga suure nurga all
    if (abs(robotAngle) > 35)
    {

      stopMotors();

      integral = 0;

      Serial.println("ROBOT FALLEN!");

      return;
    }

    // Mootoridraiveri aktiveerimine
    digitalWrite(STBY, HIGH);

    // PID regulaator
    calculatePID(dt);

    // Mootorite juhtimine
    driveMotors((int)pidOutput);

    // Andmete valjastamine
    Serial.print("Angle: ");
    Serial.print(robotAngle);

    Serial.print(" | Gyro: ");
    Serial.print(gyroY);

    Serial.print(" | PID: ");
    Serial.print(pidOutput);

    Serial.print(" | Battery: ");
    Serial.println(batteryVoltage);
  }
}
