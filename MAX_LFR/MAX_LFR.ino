#include <EEPROM.h>

enum CMD { cmdSTART,
           cmdSTOP,
           cmdREADSENS,
           cmdINFO,
           cmdCALIBR,
           cmdWAIT };
byte currentCMD = cmdWAIT;

///////////////////////////////////////Pin Announcement
const byte button = 12;
const byte led = 11;
const byte IR = 0;
const byte LMotorPwm = 5;
const byte LMotor2 = 4;
const byte LMotor1 = 7;
const byte RMotorPwm = 6;
const byte RMotor2 = 9;
const byte RMotor1 = 8;
//////////////////////////////////commands for bluetooth
const char BT_STOP = 's';
const char BT_MOVE = 'l';
const char BT_READSENS = 'r';
const char BT_INFO = 'i';
const char BT_CALIBR = 'c';
const char KP_P = 'k';
const char KD_P = 'm';
const char KP_M = 'n';
const char KD_M = 'p';
const char B_P = 'b';
const char B_M = 'd';
const char BT_TH_P = 'e';
const char BT_TH_M = 'a';
const char BT_noise_P = 'o';
const char BT_noise_M = 'x';
////////////////////////////////
const float RVIN = 10000.f;       
const float RGND = 4700.f;       
const float ADC_MAX = 1023.f;    
const float ADC_REF_VOLTS = 5.f;  
float kv = ADC_REF_VOLTS / ADC_MAX / (RGND / (RGND + RVIN));
//////////////////////////////
const byte Num_Sens = 8;
int Sens[Num_Sens]{};
int Pins[Num_Sens]{ A0, A1, A2, A3, A4, A5, A6, A7 };
int bSpeed = 180;
float kp = 0.08;
float kd = 0.9;
int ws[8] = { 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000 };
int target = 4500;
int lastPos = target;
int minSpeed = -30;
int maxSpeed = 240;
int minValue = 0;
int maxValue = 9000;
int thLine = 300;
int noise = 100;
int error;
int lastError;
int delta;
int deltaError;
int minCalibr[8]{};
int maxCalibr[8]{};
const int ADR = 100;



void Drive(int left, int right);
void LFR();
void ReadSens();
void PrintSens();
void PrintINFO();
void PrintINFO2();
void Calibration();
void LoadCalibr();
void SaveCalibr();
void LoadVariables();
byte GetBTCode();
SoftwareSerial BTSerial(3, 2);


void setup() {

  pinMode(button, INPUT_PULLUP);
  pinMode(led, OUTPUT);
  pinMode(LMotor1, OUTPUT);
  pinMode(LMotor2, OUTPUT);
  pinMode(RMotor1, OUTPUT);
  pinMode(RMotor2, OUTPUT);
  Serial.begin(9600);

  BTSerial.begin(9600);
  digitalWrite(led, HIGH);
  delay(200);
  digitalWrite(led, LOW);
  //ADCSRA |= (1 << ADPS2);
  // ADCSRA &= ~(1 << ADPS1);
  //t  ADCSRA |= (1 << ADPS0);
  LoadCalibr();
  // LoadVariables();
  
}

void loop() {
  currentCMD = cmdWAIT;
  if (currentCMD == cmdWAIT) {
    currentCMD = GetBTCode();
  }

  switch (currentCMD) {
    case cmdSTART:
      Serial.println("start");
      LFR();
      break;
    case cmdSTOP:
      Serial.println("stop");
      Drive(0, 0);
      break;
    case cmdINFO:
      PrintINFO2();
      break;
    case cmdREADSENS:
      /// Serial.println("READSENS");
      PrintSens();
      break;
    case cmdCALIBR:
      Serial.println("CALIBR");
      Calibration();
      break;
    case cmdWAIT:
      break;
  }
}

void Drive(int left, int right) {
  left = constrain(left, minSpeed, maxSpeed);
  right = constrain(right, minSpeed, maxSpeed);

  if (left >= 0) {
    digitalWrite(LMotor1, 0);
    digitalWrite(LMotor2, 1);
  } else {
    digitalWrite(LMotor1, 1);
    digitalWrite(LMotor2, 0);
  }

  if (right >= 0) {
    digitalWrite(RMotor1, 0);
    digitalWrite(RMotor2, 1);

  } else {
    digitalWrite(RMotor1, 1);
    digitalWrite(RMotor2, 0);
  }

  analogWrite(LMotorPwm, abs(left));
  analogWrite(RMotorPwm, abs(right));
}

int ReadLine() {
  long sumWV = 0;
  int sumW = 0;
  bool onLine = false;
  int distanse = 1;
  float speed = 0;



  ReadSens();

  for (int i = 0; i < Num_Sens; i++) {
    Sens[i] = map(Sens[i], minCalibr[i], maxCalibr[i], 0, 1000);
    if (Sens[i] < 0) {
      Sens[i] = 0;
    }
    if (Sens[i] > 1000) {
      Sens[i] = 1000;
    }

    if (Sens[i] > thLine) {

      onLine = true;
    }
    if (Sens[i] > noise) {
      sumWV = sumWV + (long)ws[i] * Sens[i];
      sumW = sumW + Sens[i];
    }
  }

  if (onLine == false) {
    if (lastPos < target) {
      return minValue;
    } else {
      return maxValue;
    }
  }
  //Dl,Constrain

  lastPos = sumWV / sumW;
  return lastPos;
}



void ReadSens() {
  for (int i = 0; i < Num_Sens; i++) {
    Sens[i] = analogRead(Pins[i]);
  }

  for (int i = 0; i < Num_Sens; i++) {
    Sens[i] = (Sens[i] + analogRead(Pins[i])) / 2;
  }
}

void PrintINFO() {
  for (int i = 0; i < Num_Sens; i++) {
    BTSerial.print(minCalibr[i]);
    BTSerial.print(' ');
  }

  BTSerial.println(' ');
  for (int i = 0; i < Num_Sens; i++) {
    BTSerial.print(maxCalibr[i]);
    BTSerial.print(' ');
  }

  delay(300);
}

void PrintINFO2() {
  BTSerial.print("kp=");
  BTSerial.print(" ");
  BTSerial.println(kp);
  BTSerial.print("kd=");
  BTSerial.print(" ");
  BTSerial.println(kd);
  BTSerial.print("noise=");
  BTSerial.print(" ");
  BTSerial.println(noise);
  BTSerial.print("MAX_SPEED=");
  BTSerial.print(" ");
  BTSerial.println(maxSpeed);
  BTSerial.print("MIN_SPEED=");
  BTSerial.print(" ");
  BTSerial.println(minSpeed);
  BTSerial.print("bSpeed=");
  BTSerial.print(" ");
  BTSerial.println(bSpeed);
  BTSerial.print("TH=");
  BTSerial.print(" ");
  BTSerial.println(thLine);
  BTSerial.println("///////////////////////////////////////");
  for (int i = 0; i < Num_Sens; i++) {
    BTSerial.print(minCalibr[i]);
    BTSerial.print(' ');
  }
  BTSerial.println(' ');
  for (int i = 0; i < Num_Sens; i++) {
    BTSerial.print(maxCalibr[i]);
    BTSerial.print(' ');
  }
  BTSerial.println(' ');


  delay(300);
}



void PrintSens() {
  while (1) {
    for (int i = 0; i < Num_Sens; i++) {
      ReadSens();
      BTSerial.print(Sens[i]);
      BTSerial.print(' ');
    }
    BTSerial.println();
    BTSerial.print("position:");
    BTSerial.println(ReadLine());
    delay(400);
  }
}

void Save() {
  BTSerial.println("save");
}

byte GetBTCode() {
  int retCmd = cmdWAIT;
  if (BTSerial.available()) {
    switch (BTSerial.read()) {
      case BT_MOVE:
        retCmd = cmdSTART;
        break;
      case BT_STOP:
        retCmd = cmdSTOP;
        break;
      case BT_READSENS:
        retCmd = cmdREADSENS;
        break;
      case BT_INFO:
        retCmd = cmdINFO;
        break;
      case BT_CALIBR:
        retCmd = cmdCALIBR;
        break;
      case KP_P:
        kp = kp + 0.01;
        //SaveVariables();
        break;
      case KD_P:
        kd = kd + 0.10;
        //SaveVariables();
        break;
      case KP_M:
        kp = kp - 0.01;
        //SaveVariables();
        break;
      case KD_M:
        kd = kd - 0.10;
        ///SaveVariables();
        break;
      case B_M:
        bSpeed = bSpeed - 10;
        //SaveVariables();
        break;
      case B_P:
        bSpeed = bSpeed + 10;
        ///SaveVariables();
        break;
      case BT_TH_P:
        thLine = thLine + 10;
        // SaveVariables();
        break;
      case BT_TH_M:
        thLine = thLine - 10;
        // SaveVariables();
        break;
      case BT_noise_P:
        noise = noise + 10;
        //SaveVariables();
        break;
      case BT_noise_M:
        noise = noise - 10;
        //SaveVariables();
        break;
    }
  }
  return retCmd;
}



void Calibration() {
  const int Loop = 4000;
  for (int i = 0; i < Num_Sens; i++) {
    minCalibr[i] = 1023;
  }

  for (int i = 0; i < Num_Sens; i++) {
    maxCalibr[i] = 0;
  }

  Drive(50, -50);
  for (int i = 0; i < Loop; i++) {
    ReadSens();

    for (int i = 0; i < Num_Sens; i++) {
      if (Sens[i] > maxCalibr[i]) {
        maxCalibr[i] = Sens[i];
      }

      if (Sens[i] < minCalibr[i]) {
        minCalibr[i] = Sens[i];
      }
    }
  }
  Drive(0, 0);
  SaveCalibr();
}

void LFR() {

  int error = 0;
  int last = 0;
  int delta = 0;
  int deltaError = 0;
  int lastError = 0;

  while (1) {
    if (GetBTCode() == cmdSTOP) {

      Drive(0, 0);
      return;
    }

    error = target - ReadLine();
    deltaError = error - lastError;
    lastError = error;
    delta = error * kp + deltaError * kd;
    Drive(bSpeed - delta, bSpeed + delta);
  }
}

void SaveCalibr() {
  int adr = ADR;
  for (int i = 0; i < Num_Sens; i++) {
    EEPROM.put(adr, minCalibr[i]);
    EEPROM.put(adr + 2, maxCalibr[i]);
    adr = adr + 4;
  }
}

void LoadCalibr() {
  int adr = ADR;
  for (int i = 0; i < Num_Sens; i++) {
    EEPROM.get(adr, minCalibr[i]);
    EEPROM.get(adr + 2, maxCalibr[i]);
    adr = adr + 4;
  }
}
void LoadVariables() {
  EEPROM.get(200, bSpeed);
  EEPROM.get(210, minSpeed);
  EEPROM.get(220, maxSpeed);
  EEPROM.get(230, thLine);
  EEPROM.get(240, noise);
}

void SaveVariables() {
  EEPROM.put(200, bSpeed);
  EEPROM.put(210, minSpeed);
  EEPROM.put(220, maxSpeed);
  EEPROM.put(230, thLine);
  EEPROM.put(240, noise);
}

float getVoltage() {

  static int volts[10] = {};
  static int sum = 0;
  static int i = 0;
  sum -= volts[i];
  volts[i] = analogRead(BATTERY_PIN);
  sum += volts[i];
  i++;

  if (i == 10) {
    i = 0;
  }

  return kv * ((float)sum / 10);
}
