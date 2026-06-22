#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

float gx_offset = 0, gy_offset = 0, gz_offset = 0;
float angX = 0, angY = 0, angZ = 0;

unsigned long lastTime;
unsigned long ultimoEnvio = 0;
unsigned long ultimoSonar = 0;

bool sonarVez = false;

float distE = 0;
float distF = 0;

String buffer = "";

#define TRIG_E 3
#define ECHO_E 2

#define TRIG_F 6
#define ECHO_F 5

float lerSonar(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);

  digitalWrite(trig, LOW);

  long duracao = pulseIn(echo, HIGH, 30000);

  if (duracao == 0) return -1;

  return duracao * 0.034 / 2.0;
}

void atualizarSonares() {

  if (millis() - ultimoSonar < 60)
    return;

  ultimoSonar = millis();

  if (sonarVez)
    distF = lerSonar(TRIG_F, ECHO_F);
  else
    distE = lerSonar(TRIG_E, ECHO_E);

  sonarVez = !sonarVez;
}

float normalizar(float ang) {

  while (ang > 180)
    ang -= 360;

  while (ang < -180)
    ang += 360;

  return ang;
}

void atualizar() {

  int16_t gx, gy, gz;

  mpu.getRotation(&gx, &gy, &gz);

  unsigned long now = millis();

  float dt = (now - lastTime) / 1000.0;

  lastTime = now;

  if (dt > 0.5)
    return;

  angX += ((gx - gx_offset) / 131.0) * dt;
  angY += ((gy - gy_offset) / 131.0) * dt;
  angZ += ((gz - gz_offset) / 131.0) * dt;

  angX = normalizar(angX);
  angY = normalizar(angY);
  angZ = normalizar(angZ);
}

void enviarDados() {

  Serial.print("X:");
  Serial.print(angX, 1);

  Serial.print(",Y:");
  Serial.print(angY, 1);

  Serial.print(",Z:");
  Serial.print(angZ, 1);

  Serial.print(",F:");
  Serial.print(distF, 1);

  Serial.print(",E:");
  Serial.println(distE, 1);
}

void resetarGiroscopio() {

  angX = 0;
  angY = 0;
  angZ = 0;

  lastTime = millis();
}

void piscarLED() {

  for (int i = 0; i < 3; i++) {

    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);

    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

void setup() {

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(TRIG_E, OUTPUT);
  pinMode(ECHO_E, INPUT);

  pinMode(TRIG_F, OUTPUT);
  pinMode(ECHO_F, INPUT);

  mpu.initialize();

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  delay(1000);

  long gx = 0;
  long gy = 0;
  long gz = 0;

  for (int i = 0; i < 200; i++) {

    int16_t rx, ry, rz;

    mpu.getRotation(&rx, &ry, &rz);

    gx += rx;
    gy += ry;
    gz += rz;

    delay(5);
  }

  gx_offset = gx / 200.0;
  gy_offset = gy / 200.0;
  gz_offset = gz / 200.0;

  lastTime = millis();
}

void loop() {

  atualizarSonares();

  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\n') {

      buffer.trim();

      if (buffer == "R") {
        resetarGiroscopio();
      }
      else if (buffer == "START") {
        piscarLED();
      }

      buffer = "";
    }
    else {
      buffer += c;
    }
  }

  atualizar();

  if (millis() - ultimoEnvio >= 10) {

    enviarDados();

    ultimoEnvio = millis();
  }
}