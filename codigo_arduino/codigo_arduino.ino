String buffer = "";

bool sonarVez = false;
float distE = 0;
float distF = 0;

unsigned long ultimoSonar = 0;
unsigned long ultimoEnvio = 0;

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
  if (duracao == 0) return 255;
  float dist = duracao * 0.034 / 2.0;
  if (dist < 1 || dist > 255) return 255;
  return dist;
}

void atualizarSonares() {
  if (millis() - ultimoSonar < 60) return;
  ultimoSonar = millis();
  if (sonarVez)
    distF = lerSonar(TRIG_F, ECHO_F);
  else
    distE = lerSonar(TRIG_E, ECHO_E);
  sonarVez = !sonarVez;
}

void enviarDados() {
  Serial.print("F:");
  Serial.print(distF, 1);
  Serial.print(",E:");
  Serial.println(distE, 1);
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
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIG_E, OUTPUT);
  pinMode(ECHO_E, INPUT);
  pinMode(TRIG_F, OUTPUT);
  pinMode(ECHO_F, INPUT);
}

void loop() {
  atualizarSonares();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buffer.trim();
      if (buffer == "START") piscarLED();
      buffer = "";
    } else {
      buffer += c;
    }
  }

  if (millis() - ultimoEnvio >= 10) {
    enviarDados();
    ultimoEnvio = millis();
  }
}