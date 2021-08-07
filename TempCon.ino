//#define DEBUG 1 //DEBUG用，註解掉即停止Serial

// include the library code
#include <SoftwareSerial.h>
#include <PID_v1.h>
#include <Wire.h>
SoftwareSerial BT(5, 6);
/*#include <DisplayImage.h>*/
byte degree[8] = {
  B00010,
  B00101,
  B00010,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
};

byte powerState5[8] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
};
byte powerState4[8] = {
  B00000,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
};
byte powerState3[8] = {
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
};
byte powerState2[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111,
};
byte powerState1[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
};
byte powerState0[8] = {
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
};

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
#define CURRENTT 1
#define TARGETT 2
/***Pin definitions***/
#define PIN_INPUT A0
int PIN_OUTPUT=10;
#define PIN_INPUT2 A1
#define CLK_PIN 2
#define DT_PIN 4
#define SW_PIN 3
#define INTERRUPT0 0
#define INTERRUPT1 1
#define POWERON_PIN 13 //0 as shut down, 1 as pwoer on.
#define MENUCOUNT 2
#define STEPTEMP 5
#define BTWAITTIME 1000
const int SLAVEPORT = 11;
#define POWER_FIRST 0
#define POWER_SEC 1
/*********************************************************/

#define STEPTIME 60 //定時調整的階段時間，為方便DEMO，調整為1分鐘
volatile unsigned long lastAdjustTime = 0;
#define MENUSWITCHTIME 5000
volatile int MENUStatus = 0;
volatile unsigned long closeTime = 0; //關機時間
unsigned long t = 0;
volatile int targetTemp = 250;
int currentTemp = 260;
volatile bool isUpdate = 0;
bool hasUpdate = 0;
float powerState = 300;
bool isOff = 0;
unsigned long lastByteTime = 0;
int rcvBytes[10];

/**/
bool mode;//1加熱0製冷
long long lastUpdateTime = 0;
long long lastUpdateTime2 = 0;
long long lastUpdateTime3 = 0;
double Setpoint, Input, Output,rem_Setpoint;
double KpC=41, KiC=1, KdC=0.2;
double KpH=70, KiH=1, KdH=0.7;
double KpC2=0.5, KiC2=1, KdC2=0.5;
double KpH2=0.5, KiH2=1, KdH2=0.5;
double KpC3=20, KiC3=1, KdC3=1.1;
double KpH3=35, KiH3=1, KdH3=1.1;
int overtime=0;

void modeCheck();
double Tmp;
PID PID1(&Input, &Output, &Setpoint, KpC, KiC, KdC, DIRECT); //加熱
PID PID2(&Setpoint, &Output, &Input, KpH, KiH, KdH, DIRECT); //製冷
PID PID3(&Input, &Output, &Setpoint, KpC2, KiC2, KdC2, DIRECT); //二段加熱
PID PID4(&Setpoint, &Output, &Input, KpH2, KiH2, KdH2, DIRECT); //二段製冷
PID PID5(&Input, &Output, &Setpoint, KpC3, KiC3, KdC3, DIRECT); //三段加熱
PID PID6(&Setpoint, &Output, &Input, KpH3, KiH3, KdH3, DIRECT); //三段製冷
double Vo;
float R1 = 10000;
float logR2, R2, T, Tc, Tf;
float c1 = 0.01356255169, c2 = -0.001869419441, c3 = 0.000008979517101;
int count=0;
int over=0;
/**/

void updateTemp(int temp, int row);
void updatePowerState(int state);
void rotaryEncoderChanged();

void setup()
{
  Serial.begin(9600);
 // Wire.begin(11);
 // Wire.onReceive(rcvEvent);
  /*******LCD initialize*******/
  //lcd.init();  //initialize the lcd
  lcd.init();
  lcd.backlight();  //open the backlight 
  lcd.createChar(6, degree);
  lcd.createChar(0, powerState0);
  lcd.createChar(1, powerState1);
  lcd.createChar(2, powerState2);
  lcd.createChar(3, powerState3);
  lcd.createChar(4, powerState4);
  lcd.createChar(5, powerState5);
   
  lcd.setCursor ( 0, 0 );            // go to the top left corner
  lcd.print("===< E-Mattress >==="); // write this string on the top row
  lcd.setCursor ( 0, 1 );            // go to the 2nd row
  lcd.print("Current Temp:   .  C"); // pad string with spaces for centering
  lcd.setCursor ( 0, 2 );            // go to the third row
  lcd.print("Target Temp :   .  C"); // pad with spaces for centering
  lcd.setCursor ( 18, 1 );
  lcd.write(byte(6));
  lcd.setCursor ( 18, 2 );
  lcd.write(byte(6));
  
  updatePowerState(5);
  lcd.setCursor ( 0, 3 );            // go to the fourth row
  lcd.print("Power state ");


  pinMode(CLK_PIN, INPUT_PULLUP); // 輸入模式並啟用內建上拉電阻
  pinMode(DT_PIN, INPUT_PULLUP);
  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(POWERON_PIN, OUTPUT);
  pinMode(Output, OUTPUT);
  
  digitalWrite(POWERON_PIN, 1);
  
  updateTemp(targetTemp, TARGETT);
  updateTemp(currentTemp, CURRENTT);
  
  attachInterrupt(INTERRUPT0, rotaryEncoderChanged, FALLING);
  //attachInterrupt(INTERRUPT1, switchStatus, FALLING);
  //attachInterrupt(INTERRUPT1, wakeUp, CHANGE);
  //closeTime = 3600000;
  //closeTime = 60000;
  closeTime = 0;

  /********/
  //initialize the variables we're linked to
  Input = analogRead(PIN_INPUT); //現在溫度
  Setpoint = analogRead(PIN_INPUT2); //預定溫度
  pinMode(PIN_OUTPUT, OUTPUT);
  pinMode(12, OUTPUT);

  //turn the PID on
  PID1.SetMode(AUTOMATIC);
  PID2.SetMode(AUTOMATIC);
  MENUStatus = 0;
  BT.begin(9600);
  Serial.println("BT is ready!");
}

void wakeUp() {
  //if (energy.WasSleeping()) {
    closeTime = 0;
    lcd.display();
    lcd.backlight(); // turn on backlight.
    digitalWrite(POWERON_PIN, 0);
    isOff = 0;
  //}
}

void updateTemp(int temp, int row) {
  if (temp > 100) {
    lcd.setCursor ( 14, row );
    lcd.print((temp / 10) % 100);
  } else {
    lcd.setCursor ( 14, row );
    lcd.print(" ");
    lcd.setCursor ( 15, row );
    lcd.print((temp / 10) % 10);
  }
  lcd.setCursor ( 17, row );
  lcd.print(temp % 10);
}

void updatePowerState(const int state) {
  lcd.setCursor ( 0, 3 );
  lcd.print("Power state         ");
  if (state >= 0) {
    lcd.setCursor ( 14, 3 );
    lcd.write(byte(0));
  }
  if (state > 0) {
    lcd.setCursor ( 15, 3 );
    lcd.write(byte(1));
  }
  if (state > 1) {
    lcd.setCursor ( 16, 3 );
    lcd.write(byte(2));
  }
  if (state > 2) {
    lcd.setCursor ( 17, 3 );
    lcd.write(byte(3));
  }
  if (state > 3) {
    lcd.setCursor ( 18, 3 );
    lcd.write(byte(4));
  }
  if (state > 4) {
    lcd.setCursor ( 19, 3 );
    lcd.write(byte(5));
  }
  lcd.setCursor ( 12, 3 );
  if (mode == 0) //cooldown
    lcd.write('C');
  else
    lcd.print('H');

}

void updateTiming(const unsigned long targetTime) {
  lcd.setCursor ( 0, 3 );
  lcd.print("Close after :   mins");
  lcd.setCursor ( 14, 3 );
  int remainTime = (((long long)(closeTime - millis())) / 60000.0);
  Serial.print("closeTime = ");
  Serial.println(closeTime);
  Serial.print("Menu : ");
  Serial.println(MENUStatus);
  Serial.print("Remain Time :");
  Serial.println(remainTime);
  if (remainTime > 99)
    remainTime = 99;
  lcd.print(remainTime);
}

void rotaryEncoderChanged(){ // when CLK_PIN is FALLING
  //Serial.println("1111");
  unsigned long temp = millis();
  if(temp - t < 100) // 去彈跳
    return;
  t = temp;

  // DT_PIN的狀態代表正轉或逆轉
  targetTemp += digitalRead(DT_PIN) == 1 ? STEPTEMP : -STEPTEMP;
  if (targetTemp > 320)
    targetTemp = 320;
  else if (targetTemp < 160)
  targetTemp = 160;
  isUpdate = 1;
}

double transferTemptoReadable(int iC){
  
  Vo = (158.0/138.0)*(iC+280.329113924); //0度為270,16度為424,32度為582 
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;
  return (Tc);
}

void rcvEvent() {
  #ifdef DEBUG
  Serial.println("\n\n======== < I got something > ========\n\n");
  #endif
  char rcvByte = 0;
  int rcvBytesCount = 0;
  while (Wire.available() > 0) {
      rcvByte = Wire.read();
      rcvBytes[rcvBytesCount] = rcvByte;
      rcvBytesCount ++;
  }
  #ifdef DEBUG
  Serial.print("it's : ");
  Serial.println(rcvBytes[0]);
  Serial.println((int)rcvBytes[1]);
  Serial.println(rcvBytesCount);
  #endif
/*
  //if (rcvBytesCount == 2) {
    switch (rcvBytes[0]) {
      case 'T' :
        Serial.print("it's : ");
        int temperature = rcvBytes[1] + 128;
        int tempInt = temperature % 100;
        int tempFloat = (((int)temperature / 100) == 0 ? 0 : 5);
        targetTemp = tempInt * 10 + tempFloat;
        Serial.print("Temp : ");
        Serial.println(targetTemp);
        updateTemp(targetTemp, TARGETT);
      break;
      case 'S' :
        unsigned long currentT = millis();
        int mins = rcvBytes[1] + 128;
        closeTime = currentT + (mins * 60000);
        Serial.print("Shut down after : ");
        Serial.println(closeTime);
        Serial.println(mins);
        Serial.println((unsigned long)(closeTime - currentT) / 60000);
      break;
      case 'P' :
        if (!isOff) //the machine is on >> off
          closeTime = millis() + 5000;
        else { // the machine is off >> on
          closeTime = 0;
          isOff = 0;
          digitalWrite(POWERON_PIN, 1);
          //set motor off
          lcd.backlight(); // turn off backlight
          lcd.display();
          //energy.PowerDown();
        }
      break;
      default:
        Serial.println(rcvBytes[1]);
        // do something
      break;
    }
        Serial.println(rcvBytes[1]);
    Serial.print("end");
  //}*/
}

/*void receiveEvent() {
  while (Wire.available() > 0) {
    Serial.println
    Serial.println("===========I got something!");
    Serial.println(packageCount);
    Serial.println("HAHA");
    int package = Wire.read();
    Serial.print("it's : ");

    if (packageCount == 0) {
      recvPackage.command = package;
      Serial.println(recvPackage.command);
      packageCount++;
    }
    else if (packageCount == 1) {
      recvPackage.value = package;
      Serial.println(recvPackage.value);
      packageCount++;
      switch (recvPackage.command) {
        case 'T' :
          int temperature = 25;
          temperature = recvPackage.value;
          int tempInt = (temperature % 32) % 100;
          int tempFloat = ((temperature - tempInt) / 32) % 10;
          targetTemp = tempInt * 10 + tempFloat;
          updateTemp(targetTemp, TARGETT);
        break;
        case 'S' :
          int mins = 0;
          mins = recvPackage.value;
          mins %= 255;
          closeTime =  millis() + mins * 60 * 1000;
          Serial.print("Shut down after : ");
          Serial.println((closeTime - millis() / 60000);
        break;
        case 'P' :
        if (!isOff)
          closeTime = millis() + 5000;
        else {
          closeTime = 0;
          isOff = 0;
          closeTime = 0;
          digitalWrite(POWERON_PIN, 1);
          //set motor off
          lcd.backlight(); // turn off backlight
          lcd.display();
        //energy.PowerDown();
        }
        break;
        default :
        break;
      }
      recvPackage.command = 0;
      recvPackage.value = 0;
      packageCount = 0;
    }


    
  }
}*/


/*********************************************************/
void loop() 
{
  if (rcvBytes[0] != 0) {
    switch (rcvBytes[0]) {
      case 'T' :
        Serial.print("it's : ");
        int temperature = rcvBytes[1] + 128;
        int tempInt = temperature % 100;
        int tempFloat = (((int)temperature / 100) == 0 ? 0 : 5);
        targetTemp = tempInt * 10 + tempFloat;
        if (targetTemp > 320)
          targetTemp = 320;
        else if (targetTemp < 160)
          targetTemp = 160;
        Serial.print("Temp : ");
        Serial.println(targetTemp);
        updateTemp(targetTemp, TARGETT);
      break;
      case 'S' :
        int mins = rcvBytes[1] + 128;
        closeTime = millis() + (mins * 60000);
        if (mins == 0)
          closeTime = 0;
        Serial.print("Shut down after : ");
        Serial.println(closeTime);
        Serial.println(mins);
        Serial.println((unsigned long)(closeTime - millis()) / 60000);
      break;
      case 'P' :
        if (!isOff) //the machine is on >> off
          closeTime = millis() + 2000;
        else { // the machine is off >> on
          closeTime = 0;
          isOff = 0;
          digitalWrite(POWERON_PIN, POWER_FIRST);
          digitalWrite(POWERON_PIN, POWER_SEC);
          delay(1000);
          digitalWrite(POWERON_PIN, POWER_FIRST);
          //set motor off
          lcd.backlight(); // turn off backlight
          lcd.display();
          //energy.PowerDown();
        }
      break;
      default:
        Serial.println("NONONOONONONON");
        Serial.println(rcvBytes[1]);
        // do something
      break;
    }
    rcvBytes[0] = 0;
  }
  if (BT.available()) {
    char BTRead = BT.read();
    Serial.println("===========I got something!===========");
    Serial.print("it's : ");
    Serial.print(BTRead);

    lastByteTime = millis();

    switch (BTRead) {
        case 'T':
          while(!BT.available() && millis() < BTWAITTIME + lastByteTime);
          if (BT.available()) {
            Serial.println(BTRead);
            BTRead = BT.read();
            int temperature = 25;
            temperature = BTRead + 128;
            int tempInt = temperature % 100;
            int tempFloat = (((int)temperature / 100) == 0 ? 0 : 5);
            targetTemp = tempInt * 10 + tempFloat;
            if (targetTemp > 320)
              targetTemp = 320;
            else if (targetTemp < 160)
              targetTemp = 160;
            Serial.print("Temp : ");
            Serial.println(targetTemp);
            updateTemp(targetTemp, TARGETT);
          }
          break;
        case 'S' :
          while(!BT.available() && millis() < BTWAITTIME + lastByteTime);
          if (BT.available()) {
            unsigned long currentT = millis();
            BTRead = BT.read();
            int mins = BTRead + 128;
            closeTime = currentT + (mins * 60000);
            if (mins == 0)
              closeTime = 0;
            Serial.print("Shut down after : ");
            Serial.println(closeTime);
            Serial.println(mins);
            Serial.println((unsigned long)(closeTime - currentT) / 60000);
          }
          // do something
          break;
        case 'P' :
          while(!BT.available() && millis() < BTWAITTIME + lastByteTime);
          if (BT.available()) {
            BTRead = BT.read();
            if (!isOff) //the machine is on >> off
              closeTime = millis() + 2000;
            else { // the machine is off >> on
              closeTime = 0;
              isOff = 0;
              digitalWrite(POWERON_PIN, POWER_FIRST);
              digitalWrite(POWERON_PIN, POWER_SEC);
              delay(1000);
              digitalWrite(POWERON_PIN, POWER_FIRST);
              //set motor off
              lcd.backlight(); // turn off backlight
              lcd.display();
              //energy.PowerDown();
            }
          }
        break;
        default:
          // do something
        break;
    }
  } else if (Serial.available()) {
     BT.write(Serial.read());
  }


  if (isOff)
    if (digitalRead(POWERON_PIN)) {
      isOff = 0;
      wakeUp();
    }
  
  if (closeTime != 0) {
    if (millis() > closeTime) {
      Serial.println("\n\n\n\n========Turning off!========\n\n\n\n");
      isOff = 1;
      closeTime = 0;

          digitalWrite(POWERON_PIN, POWER_FIRST);
          digitalWrite(POWERON_PIN, POWER_SEC);
          delay(1000);
          digitalWrite(POWERON_PIN, POWER_FIRST);
      //set motor off
      analogWrite(PIN_OUTPUT, 0);
      lcd.noBacklight(); // turn off backlight
      lcd.noDisplay();
      //energy.PowerDown();
    }
  }  

  if (isUpdate)
    updateTemp(targetTemp, TARGETT);
  
  
  if (millis() > lastUpdateTime + 1000 && !isOff) {
    Serial.println("");
    Input = transferTemptoReadable(analogRead(PIN_INPUT));
    Setpoint = targetTemp / 10.0;;
    if(count == 0){
      rem_Setpoint = Setpoint;
      if(Setpoint>=Input && millis() > lastUpdateTime2 + 3000){
        Setpoint = Setpoint+6;
        PID1.Compute();
        digitalWrite(12, HIGH); //加熱
        mode = 1;
        Serial.print("Mode:");
        Serial.println("Heatup");
        lastUpdateTime2 = millis();
        Output = Output/3;
        Setpoint = Setpoint-6;
      }
      if(Setpoint<Input && millis() > lastUpdateTime2 + 3000){
        Setpoint = Setpoint-1;
        PID2.Compute();
        digitalWrite(12, LOW); //製冷
        mode = 0;
        Serial.print("Mode:");
        Serial.println("Cooldown");
       lastUpdateTime2 = millis();
       Setpoint = Setpoint+1;
      }
      if(mode == 1 && Setpoint <= Input){
        count = 1;
        Serial.print("C Change a");
      }
      if(mode == 0 && Setpoint >= Input){
        count = 1;
        Serial.print("C Change b");
      }
    }
    if(count %2 == 1){
      if((Setpoint-0.3>Input || Setpoint+0.3<Input) && count != 3){
        count = 2;
        Serial.print("Return Status");
        if(mode == 1){
          PID3.Compute();
        }
        if(mode == 0){
          PID4.Compute();
        }
      }
      if((Setpoint-0.3>Input || Setpoint+0.3<Input) && count == 3){
        overtime++;
        if(overtime == 1){
          lastUpdateTime3=millis();
        }
        if(overtime >= 3 && lastUpdateTime3-7000<=millis()){
          overtime = 0;
          count = 2;
          Serial.print("Return Status");
        }
        if(overtime >= 3 && lastUpdateTime3-7000>=millis()){
          overtime = 0;
          lastUpdateTime3=millis();
        }
        if(mode == 1){
            PID3.Compute();
        }
        if(mode == 0){
            PID4.Compute();
        }
        if(rem_Setpoint <= Setpoint-0.55 || rem_Setpoint >= Setpoint+0.55){
          count = 0;
          rem_Setpoint = Setpoint;
        }
      }
    }

    if(count == 2){
      if(rem_Setpoint <= Setpoint-0.55 || rem_Setpoint >= Setpoint+0.55){
        count = 0;
        rem_Setpoint = Setpoint;
      }
      if(Setpoint>=Input && millis() > lastUpdateTime2 + 7000){
        PID5.Compute();
        digitalWrite(12, HIGH); //加熱
        mode = 1;
        Serial.print("Mode:");
        Serial.println("Heatup");
        lastUpdateTime2 = millis();
        Output = Output/3.2;
      }
      if(Setpoint<Input && millis() > lastUpdateTime2 + 7000){
        PID6.Compute();
        digitalWrite(12, LOW); //製冷
        mode = 0;
        Serial.print("Mode:");
        Serial.println("Cooldown");
        lastUpdateTime2 = millis();
      }
      if(mode == 1 && Setpoint <= Input){
        count = 3;
        Serial.print("C Change a");
        Setpoint=Setpoint+1;
        PID5.Compute();
        Setpoint=Setpoint-1;
      }
      if(mode == 0 && Setpoint >= Input){
        count = 3;
        Serial.print("C Change b");
        PID6.Compute();
      }
    }
    
    if (millis() % (MENUSWITCHTIME * 2) < MENUSWITCHTIME) {
      Serial.print("Mode:");
      Serial.println(mode);
      Serial.print("power :");
      if (mode == 1)
        updatePowerState(powerState * 5);
      else
        updatePowerState(powerState);
      Serial.println(powerState);
    } else if (closeTime != 0)
      updateTiming(closeTime);
    else {
      if (mode == 1)
        updatePowerState(powerState * 5);
      else
        updatePowerState(powerState);
      Serial.print("Mode:");
      Serial.println(mode);
      Serial.print("power :");
      Serial.println(powerState);
    }
    if(Setpoint<=32.0 && Setpoint>=16.0){
      Serial.print("Output temperature:");
      Serial.println(Output);
      powerState = Output / 51.2;
    }else{
      Serial.println("Out of safety range");
    }
    analogWrite(PIN_OUTPUT,255-Output); //輸出
    Serial.print("Current temperature:");
    Serial.println(Input);
    Serial.print("Setting temperature:");
    Serial.println(Setpoint);
    currentTemp = int(Input * 10);
    updateTemp(currentTemp, CURRENTT);
    lastUpdateTime = millis();
  }
}
/************************************************************/
