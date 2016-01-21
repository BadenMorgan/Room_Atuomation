////////////////////////////////////////////////////
//__________________________________________________

//library inclusions

////////////////////////////////////////////////////

//change to same file location so program is fully transportable
#include <Wire.h>
#include "Time.h"
#include "DS1307RTC.h"
#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_PCD8544.h"
#include "SoftReset.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "IRremote.h"
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

//timed output variables
uint8_t THourON = 25;
uint8_t TMinuteON = 20;
uint8_t THourOFF = 25;
uint8_t TMinuteOFF = 21;

//timed wake mode variables
uint8_t Whour = 8;
uint8_t Wminutes = 15;

//variables for heater relays
uint8_t heatersON = 0;

//variable to decide what should be displayed
uint8_t disp = 1;

////////////////////////////////////////////////////
//__________________________________________________

//object definitions

////////////////////////////////////////////////////

Adafruit_PCD8544 display = Adafruit_PCD8544(3, 2, A3);
// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(temp);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

//ir remote definitions
IRrecv irrecv(RECV_PIN);

decode_results results;

////////////////////////////////////////////////////
//__________________________________________________

//macros

////////////////////////////////////////////////////

#define MAINDEBUG

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
  InitDisplay();
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
    DisplaySettings();
  }
  IRreceive();
  CapCheck();
  TimedWake();
  TimedOutput();
  HeatControl();
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
void InitDisplay() {
  display.begin();
  display.setContrast(50);
  display.display(); // show splashscreen
  delay(2000);
  display.clearDisplay();   // clears the screen and buffer
}


//code that will run when the device is awake
void AwakeMode() {
  WWDMSwitchCheck();
  delay(100);
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
void WWDMSwitchCheck() {
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

  if (RTC.read(tm)) {
    hours = tm.Hour;
    minutes = tm.Minute;
    seconds = tm.Second;
    days = tm.Day;
    months = tm.Month;
    years = tmYearToCalendar(tm.Year);
    display.clearDisplay();
    display.print(F("Time:"));
    display.setCursor(0, 8);
    print2digits(hours);
    display.print(F(":"));
    print2digits(minutes);
    display.print(F(":"));
    print2digits(seconds);

    display.setCursor(0, 16);
    display.print(F("Date (D/M/Y):"));

    display.setCursor(0, 24);
    print2digits(days);
    display.print(F("/"));
    print2digits(months);
    display.print(F("/"));
    display.print(years);

    display.setCursor(0, 32);
    display.print(F("Temp:"));
    display.setCursor(0, 40);
    first2 = temperature;
    second2 = ((temperature - first2) * 100);
    print2digits(first2);
    display.print(F("."));
    print2digits(second2);
    display.display();
    delay(100);
  } else {
    if (RTC.chipPresent()) {
      display.print("run SetTime example");
      display.display();
    } else {
      display.print("RTC read error");
      display.display();
    }
    delay(9000);
    soft_restart();
  }
}

//helps print in the right format on display
void print2digits(int number) {
  if (number >= 0 && number < 10) {
    display.print(F("0"));
  }
  display.print(number);
}

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
  if (hours == THourON && minutes == TMinuteON && (expander1data & 1) == 0) {
    expander1data |= 1;
  }
  if (hours == THourOFF && minutes == TMinuteOFF && (expander1data & 1) == 1) {
    expander1data ^= 1;
  }
  UpdateExpander(1);
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
void DisplaySettings() {
  volatile uint32_t diff = millis() - Stamp3;
  if (diff >= 41000) {
    disp = 1;
  } else if (diff <= 11000) {
    display.clearDisplay();

    display.print(F("Countdown: "));
    display.print((CountDown / 1000) / 60);

    display.setCursor(0, 8);
    display.print(F("BuzzAlert: "));
    display.print(Buzz);

    display.setCursor(0, 16);
    display.print(F("Max Temp.: "));
    display.print(tempmax );

    display.setCursor(0, 24);
    display.print(F("Min Temp.: "));
    display.print(tempmin );

    display.setCursor(0, 32);
    display.print(F("Temp Cntrl: "));
    display.print(heatersON);

    display.display();
  } else if (diff <= 21000) {
    display.clearDisplay();

    display.print(F("Light Trip: "));

    display.setCursor(0, 8);
    display.print(tripval * 2);

    display.setCursor(0, 16);
    display.print(F("Current Light: "));

    display.setCursor(0, 24);
    display.print(analogRead(LDR));

    display.display();
  } else if (diff <= 31000) {

    display.clearDisplay();

    display.print(F("Hour On: "));
    display.print(THourON);

    display.setCursor(0, 8);
    display.print(F("Minute On: "));
    display.print(TMinuteON);

    display.setCursor(0, 16);
    display.print(F("Hour Off: "));
    display.print(THourOFF);

    display.setCursor(0, 24);
    display.print(F("Minute Off: "));
    display.print(TMinuteOFF);

    display.display();
  } else if (diff <= 41000) {

    display.clearDisplay();

    display.print(F("Wake hour: "));

    display.setCursor(0, 8);

    display.print(Whour);

    display.setCursor(0, 16);
    display.print(F("Wake Minute: "));

    display.setCursor(0, 24);
    display.print(Wminutes);

    display.setCursor(0, 32);
    display.print(F("    End of"));
    display.setCursor(0, 40);
    display.print(F("   Settings"));

    display.display();
  }
}

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
  if (EEPROM.read(1) != 30) {
    flag = 0;
  }
  if (EEPROM.read(2) != 78) {
    flag = 0;
  }
  if (EEPROM.read(3) != 14) {
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
  EEPROM.write(1, 30);
  EEPROM.write(2, 78);
  EEPROM.write(3, 14);
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
  display.clearDisplay();
  display.println(F("Loading"));
  display.println(F("Variables"));
  display.display();
  if (CodeCheck()) {
    EEPROM2variables();
  } else {
    clearEEPROM();
    LoadEEPROMDefaults();
  }
}

