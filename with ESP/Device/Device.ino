////////////////////////////////////////////////////
//__________________________________________________

//library inclusions

////////////////////////////////////////////////////

//change to same file location so program is fully transportable
#include <SoftwareSerial.h>
#include <Wire.h>
#include "Time.h"
#include "DS1307RTC.h"
#include <SPI.h>
#include "PCD8544_SPI.h"
#include "SoftReset.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "IRremote.h"
#include "ESP8266.h"
#include <EEPROM.h>
////////////////////////////////////////////////////
//__________________________________________________

//constant definitions

////////////////////////////////////////////////////
//window and door switch definition
#define window1 6
#define window2 7
#define door 8

//motion sensor definition
#define motion 9

//digital temperature sensor definition
#define temp 5

//I2C Expander adress definition
#define expansionadr1 0x38
#define expansionadr2 0x39

//TSOP sensor definition
#define RECV_PIN A2

//Phototransistor
#define Touch A1

//capcitive sensor
#define capSensePin 4

//LDR pin definition
#define LDR A0

////////////////////////////////////////////////////
//__________________________________________________

//variable definition

////////////////////////////////////////////////////

//LDR trip value used for Motion sensor
uint8_t  tripval = 150;

//global variable that will be used for data over mqtt
float temperature;
uint8_t hours, minutes, seconds, days, months;
uint16_t years;

//temperature min max values
uint8_t tempmax = 28;
uint8_t tempmin = 22;

//expander data
uint8_t expander1data = 0b10000000;
uint8_t expander2data = 0;

//counting variables
uint32_t Stamp = 0;

uint32_t Stamp2 = 0;
uint32_t Stamp3 = 0;

uint32_t Stamp4 = 0;

uint32_t CountDown = 7200000;//120 minutes

//boolean to see if device in awake or not
uint8_t WakeMode = 1;

//boolean to see if buzzer should be activated
uint8_t Buzz = 0;

//boolean to see if timer should be activated
uint8_t timer = 0;

//timed output variables
uint8_t THourON = 22;
uint8_t TMinuteON = 15;
uint8_t THourOFF = 23;
uint8_t TMinuteOFF = 55;

//timed wake mode variables
uint8_t Whour = 8;
uint8_t Wminutes = 15;

//variables for heater relays
uint8_t heatersON = 0;

//variable to decide what should be displayed
uint8_t disp = 1;
byte settingsdisp = 0;
////////////////////////////////////////////////////
//__________________________________________________

//macros

////////////////////////////////////////////////////
#define _DISPLAY_
//#define MAINDEBUG
#define WIFIPRESENT

////////////////////////////////////////////////////
//__________________________________________________

//object definitions

////////////////////////////////////////////////////

#ifdef _DISPLAY_
PCD8544_SPI lcd;
#endif
// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(temp);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

//ir remote definitions
IRrecv irrecv(RECV_PIN);

decode_results results;

//esp module decleration
ESP8266 esp8266(1000);


////////////////////////////////////////////////////
//__________________________________________________

//setup and main loop

////////////////////////////////////////////////////

void setup() {
#ifdef MAINDEBUG
  Serial.begin(9600);
  Serial.print("hello");
#endif

  Wire.begin();
  UpdateExpander(3);
  InitPins();
#ifdef _DISPLAY_
  InitDisplay();
  lcd.clear();
  lcd.print(F("Connecting ESP"));
#endif
#ifdef WIFIPRESENT
  esp8266.initESP8266();//connect to broker with username and password
#endif
  InitVaraibles();
  sensors.begin();
  irrecv.enableIRIn(); // Start the receiver
}

void loop() {
#ifdef MAINDEBUG
  Serial.println("beginning loop");
#endif
  //esp.process();
  if (disp) {
    TempRequest();
    DisplayInfoUpdate();
  } else {
#ifdef _DISPLAY_
    DisplaySettings();
#endif
  }
  IRreceive();
  CapCheck();
  TimedWake();
  TimedOutput();
  HeatControl();
  #ifdef WIFIPRESENT
  esp8266.MQTTProcess(SubhQue, SubExec, PublishQue);
  #endif
  if (WakeMode) {
    AwakeMode();
    RelayCounter();
    TouchButton();
    MotionCheck();
    BuzzerAlert();
  }
}

////////////////////////////////////////////////////
//__________________________________________________

//functions

////////////////////////////////////////////////////

//initializes the lcd display
#ifdef _DISPLAY_
void InitDisplay() {
  lcd.begin();
  //delay(2000);
  //display.clearDisplay();   // clears the screen and buffer
}
#endif


//code that will run when the device is awake
void AwakeMode() {
  WWDMWSwitchCheck();
  //delay(100);
}

//initialize pins
void InitPins() {
  pinMode(window1, INPUT);
  pinMode(window2, INPUT);
  pinMode(door, INPUT);
  pinMode(motion, INPUT);
}

//easy to use function to write to a given expander
void expandrwrite(uint8_t address, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(data);
  Wire.endTransmission();
}

//easy to use update function to update expanders
void UpdateExpander(uint8_t expand) {
  switch (expand) {
    case 1:
      expandrwrite(expansionadr1, expander1data);
      break;
    case 2:
      expandrwrite(expansionadr2, expander2data);
      break;
    case 3:
      expandrwrite(expansionadr1, expander1data);
      expandrwrite(expansionadr2, expander2data);
      break;
  }
}

//check windows and doors and set indicators
void WWDMWSwitchCheck() {
  //check window 1
  if (digitalRead(window1)) {
    expander2data &= 0xFE;
  } else expander2data |= 0x01;
  //check window2
  if (digitalRead(window2)) {
    expander2data &= 0xFD;
  } else expander2data |= 0x02;
  //check door
  if (digitalRead(door)) {
    expander2data &= 0xFB;
  } else expander2data |= 0x04;
  //Check motion sensor
  if (!digitalRead(motion)) {
    expander2data &= 0xF7;
  } else expander2data |= 0x08;
  if ((esp8266.connectd & 2) == 2) {
    expander2data &= 0xEF;
  } else expander2data |= 0x10;
  //update the indicators
  UpdateExpander(2);
}

//requests tempereture info from ds18b20 every second
void TempRequest() {
  if (millis() - Stamp4 >= 1000) {
    temperature = sensors.getTempCByIndex(0);
    sensors.requestTemperatures();
    Stamp4 = millis();
  }
}

//reads the rtc values and temperature values and updates the display with the new values

void DisplayInfoUpdate() {
  volatile uint8_t first2 = 0;
  volatile uint8_t second2 = 0;

  tmElements_t tm;
  byte tempseconds = seconds;
  seconds = tm.Second;
  if ((RTC.read(tm)) && (seconds != tempseconds)) {
    hours = tm.Hour;
    minutes = tm.Minute;
    days = tm.Day;
    months = tm.Month;
    years = tmYearToCalendar(tm.Year);
#ifdef _DISPLAY_
    lcd.clear();

    lcd.print(F("Time:         "));
    lcd.print(hours);
    lcd.print(F(":"));
    lcd.print(minutes);
    lcd.print(F(":"));
    lcd.print(seconds);
    lcd.print(F("      "));

    lcd.print(F("Date (D/M/Y): "));
    lcd.print(days);
    lcd.print(F("/"));
    lcd.print(months);
    lcd.print(F("/"));
    lcd.print(years);
    lcd.print(F("    "));

    lcd.print(F("Temp:         "));
    first2 = temperature;
    second2 = ((temperature - first2) * 100);
    lcd.print(first2);
    lcd.print(F("."));
    lcd.print(second2);
#endif
  } else {
#ifdef _DISPLAY_
    if (RTC.chipPresent()) {

      lcd.print(F("run SetTime example"));
    } else {
      lcd.print(F("RTC read error"));
    }
    delay(9000);
#endif
    soft_restart();
  }
}


//helps print in the right format on display
#ifdef _DISPLAY_
void print2digits(int number) {
  if (number >= 0 && number < 10) {
    lcd.print(F("0"));
  }
  lcd.print(number);
}
#endif

//ir receiver code
void IRreceive() {
  if (irrecv.decode(&results)) {
    //set 1st relay
    if (WakeMode) {
      if (results.value == 0x41BEE01F) {
        expander1data ^= 0b100000;
        Stamp = millis();
      }
      //set 2nd relay
      if (results.value == 0x41BED02F) {
        expander1data ^= 0b10000;
        Stamp = millis();
      }
      //set 3rd relay
      if (results.value == 0x41BE708F) {
        expander1data ^= 0b1000;
      }
      //set 4th relay
      if (results.value == 0x41BE40BF) {
        expander1data ^= 0b100;
      }
      //set 5th relay
      if (results.value == 0x41BEC03F) {
        expander1data ^= 0b10;
      }
      //set 6th relay
      if (results.value == 0x41BEB04F) {
        expander1data ^= 0b1;
      }
      //update values
      UpdateExpander(1);
    }
    //set daytime booleand
    if (results.value == 0x41BE609F) {
      WakeMode ^= 1;
      WakeToSleep();
    }
    if (results.value == 0x41BEA05F) {
      heatersON ^= 1;
    }

    irrecv.resume(); // Receive the next value
  }
}

//wakemode change over function
void WakeToSleep () {
  if (WakeMode) {
    expander1data = 0b10100000;
    Stamp = millis();
    UpdateExpander(1);
  } else {
    expander1data = 0;
    expander2data = 0;
    UpdateExpander(3);
    Stamp = 0;
  }
  delay(1000);
}

//Count down function that will allow relays to turn off after a certain period of time
void RelayCounter() {
  if (Stamp > 0) {
    if (millis() - Stamp >= CountDown) {
      expander1data |= 0b00110000;
      expander1data ^= 0b00110000;
      UpdateExpander(1);
      Stamp = 0;
    }
  }
}

//Touch button code for switching relays
void TouchButton() {
  volatile int touchval = analogRead(Touch);
  if (readCapacitivePin(capSensePin) > 60 && touchval < 200) {
    disp = 0;
    Stamp3 = millis();
    delay(1000);
  } else if (analogRead(Touch) < 200) {
    if (millis() - Stamp >= 250) {
      if ((expander1data & 0b00010000) == 0 && (expander1data & 0b00100000) == 0) {
        expander1data |= 0b00100000;
        Stamp = millis();
      } else if ((expander1data & 0b00010000) == 0 && (expander1data & 0b00100000)) {
        expander1data ^= 0b00100000;
        expander1data |= 0b00010000;
        Stamp = millis();
      } else if ((expander1data & 0b00010000) && (expander1data & 0b00100000) == 0) {
        expander1data |= 0b00100000;
        Stamp = millis();
      } else if ((expander1data & 0b00010000) && (expander1data & 0b00100000)) {
        expander1data ^= 0b00110000;
        Stamp = 0;
      }
      UpdateExpander(1);
      delay(100);
    }
  }
}

//motion sensor code
void MotionCheck() {
  if (digitalRead(motion) && analogRead(LDR) < (tripval * 2)) {
    Stamp = millis();
    expander1data |= 0b00100000;
    UpdateExpander(1);
  }
}

void CapCheck() {
  volatile int capval = readCapacitivePin(capSensePin);
  if (capval > 60 && analogRead(Touch) < 200) {
    disp = 0;
    Stamp3 = millis();
    delay(1000);
  } else if (capval > 60 && WakeMode) {
    WakeMode = 0;
    WakeToSleep();
    delay(1000);
  } else if (capval > 60 && !WakeMode) {
    WakeMode = 1;
    WakeToSleep();
    delay(1000);
  }
}

// readCapacitivePin
uint8_t readCapacitivePin(int pinToMeasure) {
  // This is how you declare a variable which
  //  will hold the PORT, PIN, and DDR registers
  //  on an AVR
  volatile uint8_t* port;
  volatile uint8_t* ddr;
  volatile uint8_t* pin;
  // Here we translate the input pin number from
  //  Arduino pin number to the AVR PORT, PIN, DDR,
  //  and which bit of those registers we care about.
  uint8_t bitmask;
  if ((pinToMeasure >= 0) && (pinToMeasure <= 7)) {
    port = &PORTD;
    ddr = &DDRD;
    bitmask = 1 << pinToMeasure;
    pin = &PIND;
  }
  if ((pinToMeasure > 7) && (pinToMeasure <= 13)) {
    port = &PORTB;
    ddr = &DDRB;
    bitmask = 1 << (pinToMeasure - 8);
    pin = &PINB;
  }
  if ((pinToMeasure > 13) && (pinToMeasure <= 19)) {
    port = &PORTC;
    ddr = &DDRC;
    bitmask = 1 << (pinToMeasure - 13);
    pin = &PINC;
  }
  // Discharge the pin first by setting it low and output
  *port &= ~(bitmask);
  *ddr  |= bitmask;
  delay(1);
  // Make the pin an input WITHOUT the internal pull-up on
  *ddr &= ~(bitmask);
  // Now see how long the pin to get pulled up
  int cycles = 16000;
  for (int i = 0; i < cycles; i++) {
    if (*pin & bitmask) {
      cycles = i;
      break;
    }
  }
  // Discharge the pin again by setting it low and output
  //  It's important to leave the pins low if you want to
  //  be able to touch more than 1 sensor at a time - if
  //  the sensor is left pulled high, when you touch
  //  two sensors, your body will transfer the charge between
  //  sensors.
  *port &= ~(bitmask);
  *ddr  |= bitmask;

  return cycles;
}

//buzzer temperature alert
void BuzzerAlert() {
  if (Buzz && ((temperature >= tempmax && (expander2data & 0b111) == 0 ) || (temperature <= tempmin && (expander2data & 0b111) >= 1)) && (millis() - Stamp2 >= 600000) && ((expander1data & 0b01000000) == 0)) {
    expander1data |= 0b01000000;
    UpdateExpander(1);
    Stamp2 = millis();
  }
  if ((millis() - Stamp2 >= 500) && (expander1data & 0b01000000)) {
    expander1data ^= 0b01000000;
  }
  UpdateExpander(1);
}

//memory saves for relevant settable variables
void MemorySetGet(uint8_t setGet, uint8_t var) {
  switch (setGet) {
    case 0:
      switch (var) {
        case 0:
          break;
      }
      break;
    case 1:
      switch (var) {
        case 0:
          break;
      }
      break;
  }
}

//timed output relay code
void TimedOutput() {
  if (timer) {
    if (hours == THourON && minutes == TMinuteON && (expander1data & 1) == 0) {
      expander1data |= 1;
    }
    if (hours == THourOFF && minutes == TMinuteOFF && (expander1data & 1) == 1) {
      expander1data ^= 1;
    }
    UpdateExpander(1);
  }
}

//heater relay control code
void HeatControl() {
  if (heatersON) {
    if (temperature < 24) {
      expander1data |= 0b10;
    }
    if (temperature < 23.5) {
      expander1data |= 0b100;
    }
    if (temperature < 23) {
      expander1data |= 0b1000;
    }
    if (temperature > 26) {
      expander1data |= 0b10;
      expander1data ^= 0b10;
    }
    if (temperature > 26.5) {
      expander1data |= 0b100;
      expander1data ^= 0b100;
    }
    if (temperature > 27) {
      expander1data |= 0b1000;
      expander1data ^= 0b1000;
    }
    UpdateExpander(1);
  }
}

//function that will display settings for a period of time
#ifdef _DISPLAY_
void DisplaySettings() {
  uint32_t diff = millis() - Stamp3;
  if (diff >= 41000) {
    disp = 1;
  } else if ((diff <= 11000) && (settingsdisp == 0)) {
    lcd.clear();

    lcd.print(F("Countdown: "));
    byte temp1 = (CountDown / 1000) / 60;
    if (temp1 < 10) {
      lcd.print(F("00"));
    } else if (temp1 < 100) {
      lcd.print(F("0"));
    }
    lcd.print(temp1);

    lcd.print(F("BuzzAlert: "));
    lcd.print(Buzz);
    lcd.print(F("  "));

    lcd.print(F("Max Temp.: "));
    if (tempmax < 10) {
      lcd.print(F("0"));
    }
    lcd.print(tempmax );
    lcd.print(F(" "));

    lcd.print(F("Min Temp.: "));
    if (tempmin < 10) {
      lcd.print(F("0"));
    }
    lcd.print(tempmin );
    lcd.print(F(" "));

    lcd.print(F("Temp Cntrl: "));
    lcd.print(heatersON);
    settingsdisp++;
  } else if ((diff <= 21000) && (diff >= 11000) && (settingsdisp == 1)) {
    lcd.clear();

    lcd.print(F("Light Trip:   "));

    lcd.print(tripval * 2);
    lcd.print(F("           "));

    lcd.print(F("Current Light:"));

    lcd.print(analogRead(LDR));
    settingsdisp++;
  } else if ((diff <= 31000) && (diff >= 21000) && (settingsdisp == 2)) {
    lcd.clear();

    lcd.print(F("Hour On: "));
    if (THourON < 10) {
      lcd.print(F("0"));
    }
    lcd.print(THourON);
    lcd.print(F("   "));

    lcd.print(F("Minute On: "));
    if (TMinuteON < 10) {
      lcd.print(F("0"));
    }
    lcd.print(TMinuteON);
    lcd.print(F(" "));

    lcd.print(F("Hour Off: "));
    if (THourOFF < 10) {
      lcd.print(F("0"));
    }
    lcd.print(THourOFF);
    lcd.print(F("  "));

    lcd.print(F("Minute Off: "));
    if (TMinuteOFF < 10) {
      lcd.print(F("0"));
    }
    lcd.print(TMinuteOFF);

    settingsdisp++;
  } else if ((diff <= 41000) && (diff >= 31000) && (settingsdisp == 3)) {
    lcd.clear();

    lcd.print(F("Wake hour:    "));
    lcd.print(Whour);
    if (Whour < 10) {
      lcd.print(F(" "));
    }
    lcd.print(F("            "));

    lcd.print(F("Wake Minute:  "));
    Wminutes = 8;
    lcd.print(Wminutes);
    if (Whour < 10) {
      lcd.print(F(" "));
    }
    lcd.print(F("            "));

    lcd.print(F("    End of    "));
    lcd.print(F("   Settings"));
    settingsdisp = 0;
  }
}
#endif

//wake mode for morning  without turning lights on
void TimedWake() {
  if (Whour == hours && Wminutes == minutes) {
    WakeMode = 1;
    expander1data = 0b10000000;
    UpdateExpander(1);
  }
}

//////////////////////////
//eeprom code
/////////////////////////
//if you do have problems with the default settings and see odd
//values and odd behaviour for first time use try running the
//clear rom script, its a very low chance that the unique code will
//match with what is in your micros rom but its possible
/////////////////////////
//clears eeprom
void clearEEPROM() {
  for ( int i = 0 ; i < EEPROM.length() ; i++ )
    EEPROM.write(i, 0);
}

//check first 5 bytes code is 255,30,78,13,255
//this is to check wether the decide has been
//initialized and defualts were loaded before
//the current session
uint8_t CodeCheck() {
  uint8_t flag = 1;
  if (EEPROM.read(0) != 255) {
    flag = 0;
  }
  if (EEPROM.read(1) != 35) {
    flag = 0;
  }
  if (EEPROM.read(2) != 72) {
    flag = 0;
  }
  if (EEPROM.read(3) != 64) {
    flag = 0;
  }
  if (EEPROM.read(4) != 255) {
    flag = 0;
  }
  return flag;
}

//load defaults into eeprom
void LoadEEPROMDefaults() {
  //unique code for identifying first run
  EEPROM.write(0, 255);
  EEPROM.write(1, 35);
  EEPROM.write(2, 72);
  EEPROM.write(3, 64);
  EEPROM.write(4, 255);
  //default values for
  uint8_t tempcount = (CountDown / 1000) / 60;
#ifdef MAINDEBUG
  Serial.println("loaded:");
  Serial.println(tempcount);
#endif
  EEPROM.write(5, tempcount);
  EEPROM.write(6, Buzz );
  EEPROM.write(7, WakeMode );
  EEPROM.write(8, tempmax );
  EEPROM.write(9, tempmin );
  EEPROM.write(10, tripval );
  EEPROM.write(11, THourON );
  EEPROM.write(12, TMinuteON );
  EEPROM.write(13, THourOFF );
  EEPROM.write(14, TMinuteOFF );
  EEPROM.write(15, heatersON );
  EEPROM.write(16, Whour );
  EEPROM.write(17, Wminutes );
}

//load varaible from eeprom
void EEPROM2variables() {
  uint32_t tempcount = EEPROM.read(5);
  tempcount = tempcount * 1000 * 60;
#ifdef MAINDEBUG
  Serial.println("retrieved:");
  Serial.println(tempcount);
#endif
  CountDown = tempcount;
  Buzz = EEPROM.read(6);
  WakeMode = EEPROM.read(7);
  tempmax = EEPROM.read(8);
  tempmin = EEPROM.read(9);
  tripval = EEPROM.read(10);
  THourON = EEPROM.read(11);
  TMinuteON = EEPROM.read(12);
  THourOFF = EEPROM.read(13);
  TMinuteOFF = EEPROM.read(14);
  heatersON = EEPROM.read(15);
  Whour = EEPROM.read(16);
  Wminutes = EEPROM.read(17);
}

//initialize variables
void InitVaraibles() {
#ifdef _DISPLAY_
  lcd.clear();
  lcd.print(F("Loading       Variables"));
#endif
  if (CodeCheck()) {
    EEPROM2variables();
  } else {
    clearEEPROM();
    LoadEEPROMDefaults();
  }
}

void PublishQue() {
  //publish a message using an array
  byte tempyears = (byte) (years - 2000);
  byte first2 = temperature;
  byte second2 = (byte) ((temperature - first2) * 100);
  byte indicator1 = expander2data & 0x07;
  indicator1 ^= WakeMode << 3;
  indicator1 |= WakeMode << 3;
  indicator1 ^= Buzz << 4;
  indicator1 |= Buzz << 4;
  indicator1 ^= heatersON << 5;
  indicator1 |= heatersON << 5;
  byte msg[11] = {0x01, hours, minutes, seconds, days, months, tempyears, first2, second2, indicator1, expander1data};                                //message payload of MQTT package, put your payload here
  String topic = "d/0";                                  //topic of MQTT package, put your topic here
  esp8266.MQTTPublish(topic, &msg[0], 10);
}

/////////////////////////
//Subscribe topics list
////////////////////////
void SubhQue() {
  esp8266.MQTTSubscribe("c/0");                             //put your subs here with corresponding topics
}

////////////////////////////////////////
//ESP subscription handling function
///////////////////////////////////////
void SubExec() {
  //IGNORE
  ////////////////////////////////
  /*if (esp8266.Sub1->len > 0) { //receivedmsg != "") {
    if (( ((String)(esp8266.Sub1->topic)) == "hello") && (esp8266.Sub1->payloadlen == 4)) {
      for (int i = 0 ; i < 4 ; i++ ) {
        digitalWrite(2 + i, (byte)(esp8266.Sub1->payload[i]) - 48);
      }
    }
    esp8266.Sub1->len = 0;
  }*/
}
