#include <EEPROM.h>
#include <Servo.h>
Servo s;
enum CMD { cmdSTART,
           cmdSTOP,
           cmdREADSENS,
           cmdINFO,
           cmdCALIBR,
           cmdWAIT,
           cmdTurboOn,
           cmdTurboOff,
           cmdTestDrive
            };
byte currentCMD = cmdWAIT;

///////////////////////////////////////Pin Announcement
const byte button = 12;
const byte led = 3;
const byte IR = 0;

const byte LMotor2 = 4;
const byte LMotor1 = 5;

const byte RMotor2 = 28;
const byte RMotor1 = 29;
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
const char BT_TurboOn = 'y';
const char BT_TurboOff = 'z';
const char BT_TurboSpeedP = 'g';
const char BT_TurboSpeedM = 'q';
const char BT_TestDrive = 't';
////////////////////////////////
const float RVIN = 10000.f;       
const float RGND = 4700.f;       
const float ADC_MAX = 1023.f;    
const float ADC_REF_VOLTS = 5.f;  
float kv = ADC_REF_VOLTS / ADC_MAX / (RGND / (RGND + RVIN));
//////////////////////////////
const byte Num_Sens = 11;
int TurboSpeed=1400;
int Sens[Num_Sens]{};
int bSpeed = 122;
float kp = 0.06;
float kd = 3.0;
int ws[11] = { 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000 ,9000,10000,11000};
int target = 6000;
int lastPos = target;
int minSpeed = 0;
int maxSpeed = 150;
int minValue = 0;
int maxValue = 12000;
int thLine = 457;
int noise = 150;
int error;
int lastError;
int delta;
int deltaError;
int minCalibr[11]{};
int maxCalibr[11]{};
const int ADR = 100;

int S1 = 10;
int S2 = 11;
int S3 = 12;
int S4 = 13;
int analog = 26;

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
void TestDrive();

void setup() {

  pinMode(button, INPUT_PULLUP);
  pinMode(led, OUTPUT);
  pinMode(LMotor1, OUTPUT);
  pinMode(LMotor2, OUTPUT);
  pinMode(RMotor1, OUTPUT);
  pinMode(RMotor2, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(S4, OUTPUT);
  Serial.begin(9600);

  Serial2.begin(9600);
  digitalWrite(led, HIGH);
  delay(200);
  digitalWrite(led, LOW);
 
  pinMode(2, OUTPUT);
  s.attach(2);
  pinMode(1, INPUT_PULLUP);
 
  s.writeMicroseconds(1000);
 
  
}

void loop() {
  currentCMD = cmdWAIT;
  if (currentCMD == cmdWAIT) {
    currentCMD = GetBTCode();
  }

  switch (currentCMD) {
    case cmdSTART:
      LFR();
      break;
    case cmdSTOP:
     
      Drive(0, 0);
      break;
    case cmdINFO:
      PrintINFO2();
      break;
    case cmdREADSENS:
    
      PrintSens();
      break;
    case cmdCALIBR:
     // Serial.println("CALIBR");
      Calibration();
      break;
      case cmdTurboOn:
      TurboOn();
      break;
      case cmdTurboOff:
      TurboOff();
      break;
    case cmdWAIT:
      break;
  }
}


void Drive(int left, int right) {
  left = constrain(left, minSpeed, maxSpeed);
  right = constrain(right, minSpeed, maxSpeed);
  if (left >= 0) {
    digitalWrite(LMotor1, HIGH);
    analogWrite(LMotor2, 255 - left);
  } else {
    digitalWrite(LMotor2, HIGH);
    analogWrite(LMotor1, 255 + left);
  }
  if (right >= 0) {
    digitalWrite(RMotor1, HIGH);
    analogWrite(RMotor2, 255 - right);
  } else {
    digitalWrite(RMotor2, HIGH);
    analogWrite(RMotor1, 255 + right);
  }
}


int ReadLine() {
  long sumWV = 0;
  int sumW = 0;
  bool onLine = false;
  int distanse = 1;
  float speed = 0;



  ReadSens();

  for (int i = 0; i < Num_Sens; i++) {
    //Sens[i] = map(Sens[i], minCalibr[i], maxCalibr[i], 0, 1000);
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
    if (lastPos < 5000) {
      return minValue;
    } else if(lastPos > 7000) {
      return maxValue;
    }
    else{
    return target;
    }
  }
  //Dl,Constrain
    
  lastPos = sumWV / sumW;
  return lastPos;
}



void ReadSens() {
   for (int i = 5; i <= 15; i++) {
    digitalWrite(S1, i & 1);
    digitalWrite(S2, (i >> 1) & 1);
    digitalWrite(S3, (i >> 2) & 1);
    digitalWrite(S4, (i >> 3) & 1);
    delayMicroseconds(20);
    Sens[i - 5] = analogRead(analog);
  }
}

void PrintINFO() {
  for (int i = 0; i < Num_Sens; i++) {
    Serial2.print(minCalibr[i]);
    Serial2.print(' ');
  }

  Serial2.println(' ');
  for (int i = 0; i < Num_Sens; i++) {
    Serial2.print(maxCalibr[i]);
    Serial2.print(' ');
  }

  delay(300);
}

void PrintINFO2() {
  Serial2.print("kp");
  Serial2.print(" |");
  Serial2.println(kp);
  Serial2.print("kd");
  Serial2.print(" |");
  Serial2.println(kd);
  Serial2.print("noise");
  Serial2.print(" |");
  Serial2.println(noise);
  Serial2.print("MAX_SPEED");
  Serial2.print(" |");
  Serial2.println(maxSpeed);
  Serial2.print("MIN_SPEED");
  Serial2.print(" |");
  Serial2.println(minSpeed);
  Serial2.print("bSpeed");
  Serial2.print(" |");
  Serial2.println(bSpeed);
   Serial2.print("TurboSpeed");
  Serial2.print(" |");
  Serial2.println(TurboSpeed);
  Serial2.print("TH");
  Serial2.print(" |");
  Serial2.println(thLine);
  Serial2.println("___________________");
  for (int i = 0; i < Num_Sens; i++) {
    Serial2.print(minCalibr[i]);
    Serial2.print(' ');
  }
  Serial2.println(' ');
  for (int i = 0; i < Num_Sens; i++) {
    Serial2.print(maxCalibr[i]);
    Serial2.print(' ');
  }
  Serial2.println(' ');


  
}



void PrintSens() {
  while (1) {
    Serial2.print(" | ");
    for (int i = 0; i < Num_Sens; i++) {
      ReadSens();
      Serial2.print(Sens[i]);
      Serial2.print(" | ");
    }
    Serial2.println();
    Serial2.print("position:");
    Serial2.println(ReadLine());
    delay(400);
  }
}


byte GetBTCode() {
  int retCmd = cmdWAIT;
  if (Serial2.available()) {
    switch (Serial2.read()) {
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
        Serial2.print("kp ");
         Serial2.println(kp);
        break;
      case KD_P:
      Serial2.print("kd ");
        kd = kd + 0.10;
        Serial2.println(kd);
        break;
      case KP_M:
      Serial2.print("kp ");
        kp = kp - 0.01;
         Serial2.println(kp);
        break;
      case KD_M:
      Serial2.print("kd ");
        kd = kd - 0.10;
         Serial2.println(kd);
        break;
      case B_M:
      Serial2.print("Speed ");
        bSpeed = bSpeed - 5;
         Serial2.println(bSpeed);
        break;
      case B_P:
      Serial2.print("Speed ");
        bSpeed = bSpeed + 5;
         Serial2.println(bSpeed);
        break;
      case BT_TH_P:
      Serial2.print("TH ");
        thLine = thLine + 10;
        Serial2.println(thLine);
        break;
      case BT_TH_M:
      Serial2.print("TH ");
        thLine = thLine - 10;
         Serial2.println(thLine);
        break;
      case BT_noise_P:
      Serial2.print("noise ");
        noise = noise + 5;
      Serial2.println(noise);
        break;
      case BT_noise_M:
      Serial2.print("noise ");
        noise = noise - 5;
         Serial2.println(noise);
        break;
        case BT_TurboOn:
        retCmd=cmdTurboOn;
        break;
        case BT_TurboOff:
        retCmd = cmdTurboOff;
        break;
        case BT_TurboSpeedP:
        Serial2.print("TurboSpeed ");
        TurboSpeed += 10;
         Serial2.println(TurboSpeed);
        break;
        case BT_TurboSpeedM:
         Serial2.print("TurboSpeed ");
         TurboSpeed -= 10;
          Serial2.println(TurboSpeed);
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


float getVoltage() {

  static int volts[10] = {};
  static int sum = 0;
  static int i = 0;
  sum -= volts[i];
  volts[i] = analogRead(14);
  sum += volts[i];
  i++;

  if (i == 10) {
    i = 0;
  }

  return kv * ((float)sum / 10);
}
void TurboOn(){
  for(int i = 1000; i < TurboSpeed;i++){
  s.writeMicroseconds(i);
  delay(10);
  }
  
}
void TurboOff(){
  s.writeMicroseconds(1000);
}

void TestDrive(){
    while(1){   
                if (GetBTCode() == cmdSTOP) {

      Drive(0, 0);
      return;
    }

drive(50,50);
    }
}
