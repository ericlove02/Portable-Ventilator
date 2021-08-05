/*
 * PVent.ino
 */

//direction values
#define INH 1
#define EXH -1

#define SET_LOW(x,y) (x&=(~(1<<y)))
#define SET_HIGH(x,y) (x|=(1<<y))

//* PORTD
#define D0  0
#define D1  1
#define D2  2
#define D3  3 // ON A TIMER
#define D4  4
#define D5  5 // ON A TIMER
#define D6  6 // ON A TIMER
#define D7  7
//* PORTB
#define D8  0
#define D9  1 // ON A TIMER
#define D10  2 // ON A TIMER
#define D11  3 // ON A TIMER
#define D12  4
#define D13  5

/********* CONFIGURATION *********/
//all configurable values should be defined here
#define MAX_STEPS 275 //the max number of int steps motor can run without physically breaking the system
#define MICROSTEP 1 // driver = 1/rev microsteps
#define MAX_MICROSTEPS (MAX_STEPS * MICROSTEP)

#define DEBUG 1 //0=off, 1=on. enables serial output and any other debug stuff.
#define BAUD_RATE 9600

double pressureWarning = 3.5;
int resStartBPM = 20;

const int homeSwitch = 7; //limit switch
const int dir_pin = D8;
const int step_pin = D9;
const int pressurePin = 0;
const int buzzerPin = 13;

//pot minimum values
//if the pot is lower than these values, do something else, i.e. turn off, responsive breathing, etc.
#define MIN_BPM 6 //when under responsive breathing. //TODO: responsive breathing
#define MAX_BPM 60

/********* END CONFIGURATION *********/

#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <RotaryEncoder.h>

int selectedRow = 1;
String currentMenu = "mode";
int submenu = 0;
int rowsInMenu = 2;
int prevSelectedRow;
boolean cleared = true;
int prevSubmenu;

const int potButton = 5;
int buttonState;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
boolean buttonClicked;
boolean buttonUseState = false;

int stepsPerHalfBreath = 20;
double ItoE = 1;
int breathsPerMinute = 20;
int peep = 5;
int dir = EXH;

int rawPressure;
double pressureOffset = 51.0;
double pressureFull = 1023.0;
double pressure;
double pressureConverted;

int lastBpm = 0;
int lastStep = 0;
int lastIE = 0;
bool breathEnabled = true; // for responsive ONLY
int setSteps = 0;

volatile uint16_t compare_time = 113;

unsigned long speed;
unsigned long NextTime = 0;
int PulseStatus = 0;
volatile int halfStepsElapsed = 0;

void timerOn() {
  TCCR1B |= (1 << CS12);
  TCCR1B &= ~(1 << CS11);
  TCCR1B &= ~(1 << CS10);
}
void timerOff() {
  TCCR1B &= ~(1 << CS12);
  TCCR1B &= ~(1 << CS11);
  TCCR1B &= ~(1 << CS10);
}

RotaryEncoder encoder(A2, A3);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void mandatoryMode() {
  long stepsPerMin = 2.0 * long(stepsPerHalfBreath) * long(breathsPerMinute);
  double stepsPerSec = stepsPerMin / 60.0;
  double newSpeed = (1 / stepsPerSec); //seconds per step. not really a speed more like inverse of speed. The time for how long one step should take.
  if (ItoE != 1) {
    double timePerOneBreath = 60.0 / breathsPerMinute;
    double timeExhale = (timePerOneBreath * ItoE) / (1.0 + ItoE);

    double timeInhale = timePerOneBreath - timeExhale;
    double newStepsPerSec;
    if (dir == INH) {
      newStepsPerSec = (timePerOneBreath * stepsPerSec) / (2 * timeInhale);
    }
    else {
      newStepsPerSec = (timePerOneBreath * stepsPerSec) / (2 * timeExhale);
    }
    newSpeed = (1 / newStepsPerSec);
  }
  if (newSpeed <= .002)
    newSpeed = .002; //? ERICS ItoE FIX!
  compare_time = newSpeed * 16e6 / 256 / 2;

  if (dir == INH && halfStepsElapsed / 2 >= stepsPerHalfBreath) {
    halfStepsElapsed = 0;
    SET_HIGH(PORTB, dir_pin); // digitalWrite(dir_pin, HIGH);
    dir = EXH;
  }
  else if (dir == EXH && !digitalRead(homeSwitch)) {
    halfStepsElapsed = 0;
    SET_LOW(PORTB, dir_pin); // digitalWrite(dir_pin, LOW);
    dir = INH;
  }
}

void responsiveMode() {
  long stepsPerMin = 2.0 * long(stepsPerHalfBreath) * long(breathsPerMinute);
  double stepsPerSec = stepsPerMin / 60.0;
  double newSpeed = (1 / stepsPerSec); //seconds per step. not really a speed more like inverse of speed. The time for how long one step should take.

  if (ItoE != 1) {
    double timePerOneBreath = 60.0 / breathsPerMinute;
    double timeExhale = (timePerOneBreath * ItoE) / (1.0 + ItoE);

    double timeInhale = timePerOneBreath - timeExhale;
    double newStepsPerSec;
    if (dir == INH) {
      newStepsPerSec = (timePerOneBreath * stepsPerSec) / (2 * timeInhale);
    }
    else {
      newStepsPerSec = (timePerOneBreath * stepsPerSec) / (2 * timeExhale);
    }
    newSpeed = (1 / newStepsPerSec);
  }
  if (newSpeed <= .002)
    newSpeed = .002; //? ERICS ItoE FIX!
  compare_time = newSpeed * 16e6 / 256 / 2;

  if (dir == INH && halfStepsElapsed / 2 >= stepsPerHalfBreath && breathEnabled) {
    halfStepsElapsed = 0;
    SET_HIGH(PORTB, dir_pin); // digitalWrite(dir_pin, HIGH);
    dir = EXH;
    setSteps = 0;
  }
  if (dir == EXH && !digitalRead(homeSwitch) && breathEnabled) {
    halfStepsElapsed = 0;
    SET_LOW(PORTB, dir_pin); // digitalWrite(dir_pin, LOW);
    dir = INH;
    breathEnabled = false;
    timerOff();
  }
  //println("Pressure con: " + String(pressureConverted));
  if (pressureConverted < peep && (peep - pressureConverted) >= 1.5 && !breathEnabled) {
    breathEnabled = true;
    timerOn();
  }

  /*  Monitor patient pressure, pressure will stay at one number until
     volume in patient lungs increases to go for a breath
      Once pressure drops below user set peep -> begin INH of size user set volume(steps)
      EXH following the i:e ratio
      Stay at rest until pressure drops below peep again
      If pressure goes above a certain (hard coded?) amount, sound alarm
      TODO: adjust speed based on patients breath cycle,
       if patient INH during EXH, speed up rate,
       ?? if patient holds at home too long, slow down rate
           ?? when is correct to slow down speed
  */
  /*
     tested, does not work
      // if patient INH during EXH
      if(pressureConverted < peep && (peep - pressureConverted) >= 1.5 && breathEnabled && dir == EXH){
        SET_LOW(PORTB,dir_pin); // digitalWrite(dir_pin, LOW);
        dir = INH;
        setSteps = halfStepsElapsed;
        println(String(setSteps));
        halfStepsElapsed = 0;
      }
      // if pressure builds up to limit, begin exh
      if(pressure > PRESSURE_WARNING && breathEnabled && dir == INH){
        SET_HIGH(PORTB,dir_pin); // digitalWrite(dir_pin, HIGH);
        dir = EXH;
        setSteps = halfStepsElapsed;
        halfStepsElapsed = 0;
      }
  */
}

void lcdPrint(String inStr, int col = 0, int row = 0, int size = 0) {
  const char *in = inStr.c_str();
  lcd.setCursor(col, row);
  int i = 0;
  for (; in[i] != 0; i++) {
    if (in[i] == '\n') {
      col = 0;
      lcd.setCursor(col, ++row);
    } else {
      lcd.write(in[i]);
      lcd.setCursor(++col, row);
    }
  }
  for (; i < size; i++) {
    lcd.write(' ');
    lcd.setCursor(++col, row);
  }
}
void lcdPrintSimp(String str, int col, int row) {
  lcd.setCursor(col, row);
  lcd.print(str);
}
void lcdWrite(char chr, int col, int row) {
  lcd.setCursor(col, row);
  lcd.write(chr);
}

void stepperHome(String message) {
  lcd.clear();
  lcdPrint(message, 0, 0, 0);
  int homeSteps = 130;
  int homeBPM = 30;
  long stepsPerMin = 2.0 * long(homeSteps) * long(homeBPM);
  double stepsPerSec = stepsPerMin / 60.0;
  double newSpeed = (1 / stepsPerSec);
  compare_time = newSpeed * 16e6 / 256 / 2;
  while (digitalRead(homeSwitch)) {
    SET_HIGH(PORTB, dir_pin);
    dir = EXH;
    timerOn();
  }
  timerOff();
  lcd.clear();
}

byte unchecked[8] = {B00000, B00000, B11111, B10001, B10001, B11111, B00000, B00000,};
byte checked[8] = {B00000, B00000, B11111, B11111, B11111, B11111, B00000, B00000,};
byte upArrow[8] = {B00000, B00000, B00000, B00100, B01010, B10001, B00000, B00000,};
byte bothArrow[8] = {B00100, B01010, B10001, B00000, B00000, B10001, B01010, B00100,};
byte downArrow[8] = {B00000, B00000, B00000, B10001, B01010, B00100, B00000, B00000,};
byte back[8] = {B00000, B11111, B11101, B11011, B10111, B11011, B11101, B11111,};

void setup() {

  cli(); //temp disable interrupts
  TCCR1A = (1 << COM1A0); //turns off all bits except COM1A0 is turned on. Enables auto toggle of OC1A aka pin 9
  TCCR1B &= ~(1 << WGM13); //set up CTC
  TCCR1B |= (1 << WGM12); //set up CTC cont.

  //set prescalar to 256
  TCCR1B |= (1 << CS12);
  TCCR1B &= ~(1 << CS11);
  TCCR1B &= ~(1 << CS10);

  TCNT1 = 0; //reset timer to 0
  OCR1A = 23437; //set timer compare value. This number will be overwritten in later code anyway
  TIMSK1 |= (1 << OCIE1A); //enables compare A interrupt
  sei(); //enable global interrupts

  Serial.begin(BAUD_RATE);
  Serial.write("INIT!\n");

  PCICR |= (1 << PCIE1);    // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);  // This enables the interrupt for pin 2 and 3 of Port C.

  lcd.init();
  lcd.backlight();

  lcd.createChar(0, unchecked);
  lcd.createChar(1, checked);
  lcd.createChar(2, downArrow);
  lcd.createChar(3, upArrow);
  lcd.createChar(4, bothArrow);
  lcd.createChar(5, back);

  SET_HIGH(DDRB, dir_pin); // pinMode(dir_pin,OUTPUT);
  SET_HIGH(DDRB, step_pin); // pinMode(step_pin,OUTPUT);
  pinMode(homeSwitch, INPUT_PULLUP);
  SET_HIGH(PORTB, dir_pin); // digitalWrite(dir_pin,HIGH); // LOW is inhale
  pinMode(potButton, INPUT);
  pinMode(buzzerPin, OUTPUT);

  stepperHome("P-Vent\nv1.0");

  encoder.setPosition(0);
}

ISR(PCINT1_vect) {
  encoder.tick(); // just call tick() to check the state.
}

long lastTime = 0;

void loop() {
  //Serial.println("Loop entered");
  encoder.tick();
  int buttonRead = digitalRead(potButton);
  if (buttonRead != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (buttonRead != buttonState) {
      buttonState = buttonRead;
      if (buttonState == LOW) {
        buttonClicked = true;
        //Serial.println("click");
      }
    }
  }
  //Serial.println(encoder.getPosition());

  rawPressure = analogRead(pressurePin);
  pressure = (rawPressure - pressureOffset) * 10 / (pressureFull - pressureOffset); // converted to kPa
  pressureConverted = pressure * 10.197;

  if (pressure >= pressureWarning) {
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(buzzerPin, LOW);
  }
  else {
    digitalWrite(buzzerPin, LOW);
  }

  if (currentMenu.equals("mode")
      || currentMenu.equals("mandatory")
      || currentMenu.equals("responsive")
      || currentMenu.equals("settings")) {
    selectedRow = encoder.getPosition();
    if (selectedRow > rowsInMenu) {
      selectedRow = rowsInMenu;
      encoder.setPosition(rowsInMenu);
    }
    else if (selectedRow < 1) {
      selectedRow = 1;
      encoder.setPosition(1);
    }
  }

  if (prevSelectedRow != selectedRow || prevSubmenu != submenu) {
    cleared = false;
  }

  if (currentMenu.equals("mode")) {
    rowsInMenu = 3;
    if (selectedRow == 1) {
      if (!cleared) {
        lcd.clear();
        cleared = true;
      }
      lcdWrite((selectedRow == 1) ? byte(1) : byte(0), 0, 0);
      lcdPrintSimp("Mandatory", 1, 0);
      lcdWrite((selectedRow == 2) ? byte(1) : byte(0), 0, 1);
      lcdPrintSimp("Responsive", 1, 1);
      lcdWrite(byte(2), 15, 1);
    }
    else if (selectedRow == 2 || selectedRow == 3) {
      if (!cleared) {
        lcd.clear();
        cleared = true;
      }
      lcdWrite((selectedRow == 2) ? byte(1) : byte(0), 0, 0);
      lcdPrintSimp("Responsive", 1, 0);
      lcdWrite((selectedRow == 3) ? byte(1) : byte(0), 0, 1);
      lcdPrintSimp("Settings", 1, 1);
      lcdWrite(byte(3), 15, 1);
    }
    if (buttonClicked && selectedRow == 1) {
      buttonClicked = false;
      currentMenu = "mandatory";
      lcd.clear();
      encoder.setPosition(1);
    }
    else if (buttonClicked && selectedRow == 2) {
      buttonClicked = false;
      currentMenu = "responsive";
      lcd.clear();
      encoder.setPosition(1);
    }
    else if (buttonClicked && selectedRow == 3) {
      buttonClicked = false;
      currentMenu = "settings";
      lcd.clear();
      encoder.setPosition(1);
    }
  }

  else if (currentMenu.equals("mandatory")) {
    rowsInMenu = 5;
    if (selectedRow == 1) {
      if (!cleared) {
        lcd.clear();
        cleared = true;
      }
      lcdWrite((selectedRow == 1) ? byte(1) : byte(0), 0, 0);
      lcdPrintSimp("Volume: " + String(stepsPerHalfBreath), 1, 0);
      lcdWrite((selectedRow == 2) ? byte(1) : byte(0), 0, 1);
      lcdPrintSimp("Rate: " + String(breathsPerMinute), 1, 1);
      lcdWrite(byte(2), 15, 1);
    }
    else if (selectedRow == 2) {
      if (!cleared) {
        lcd.clear();
        cleared = true;
      }
      lcdWrite((selectedRow == 2) ? byte(1) : byte(0), 0, 0);
      lcdPrintSimp("Rate: " + String(breathsPerMinute), 1, 0);
      lcdWrite((selectedRow == 3) ? byte(1) : byte(0), 0, 1);
      lcdPrintSimp("I/E: 1:" + String(ItoE).substring(0, 1), 1, 1);
      lcdWrite(byte(4), 15, 1);
    }
    else if (selectedRow == 3) {
      if (!cleared) {
        lcd.clear();
        cleared = true;
      }
      lcdWrite((selectedRow == 3) ? byte(1) : byte(0), 0, 0);
      lcdPrintSimp("I/E: 1:" + String(ItoE).substring(0, 1), 1, 0);
      lcdWrite((selectedRow == 4) ? byte(1) : byte(0), 0, 1);
      lcdPrintSimp("START", 1, 1);
      lcdWrite(byte(4), 15, 1);
    }
    else if (selectedRow == 4 || selectedRow == 5) {
      if (!cleared) {
        lcd.clear();
        cleared = true;
      }
      lcdWrite((selectedRow == 4) ? byte(1) : byte(0), 0, 0);
      lcdPrintSimp("START", 1, 0);
      lcdWrite((selectedRow == 5) ? byte(1) : byte(0), 0, 1);
      lcdPrintSimp("BACK", 1, 1);
      lcdWrite(byte(3), 15, 1);
    }
    if (buttonClicked && selectedRow == 1) {
      buttonClicked = false;
      encoder.setPosition(stepsPerHalfBreath / 5);
      currentMenu = "volumeMan";
    }
    else if (buttonClicked && selectedRow == 2) {
      buttonClicked = false;
      encoder.setPosition(breathsPerMinute / 2);
      currentMenu = "rateMan";
    }
    else if (buttonClicked && selectedRow == 3) {
      buttonClicked = false;
      encoder.setPosition(ItoE);
      currentMenu = "ItoEMan";
    }
    else if (buttonClicked && selectedRow == 4) {
      buttonClicked = false;
      encoder.setPosition(0);
      currentMenu = "runningMan";
      lcd.clear();
    }
    else if (buttonClicked && selectedRow == 5) {
      buttonClicked = false;
      encoder.setPosition(1);
      currentMenu = "mode";
      lcd.clear();
    }
  }

  else if (currentMenu.equals("volumeMan")
           || currentMenu.equals("rateMan")
           || currentMenu.equals("ItoEMan")) {
    lcdWrite(' ', 0, 0);
    lcdWrite(' ', 0, 1);
    if (currentMenu.equals("volumeMan")) {
      lcdWrite(byte(1), 8, 0);
      stepsPerHalfBreath = encoder.getPosition() * 5;
      if (stepsPerHalfBreath < 20) {
        stepsPerHalfBreath = 20;
        encoder.setPosition(4);
      }
      if (stepsPerHalfBreath > 275) {
        stepsPerHalfBreath = 275;
        encoder.setPosition(55);
      }
      lcdPrint(String(stepsPerHalfBreath), 9, 0, 4);
    }
    else if (currentMenu.equals("rateMan")) {
      lcdWrite(byte(1), 6, 0);
      breathsPerMinute = encoder.getPosition() * 2;
      if (breathsPerMinute < 6) {
        breathsPerMinute = 6;
        encoder.setPosition(3);
      }
      if (breathsPerMinute > 60) {
        breathsPerMinute = 60;
        encoder.setPosition(30);
      }
      lcdPrint(String(breathsPerMinute), 7, 0, 2);
    }
    else if (currentMenu.equals("ItoEMan")) {
      lcdWrite(byte(1), 5, 0);
      ItoE = encoder.getPosition();
      if (ItoE < 1) {
        ItoE = 1;
        encoder.setPosition(1);
      }
      if (ItoE > 4) {
        ItoE = 4;
        encoder.setPosition(4);
      }
      lcdPrint(String(ItoE).substring(0, 1), 8, 0);
    }
    if (buttonClicked) {
      if (currentMenu.equals("volumeMan")) {
        encoder.setPosition(1);
      }
      else if (currentMenu.equals("rateMan")) {
        encoder.setPosition(2);
      }
      else if (currentMenu.equals("ItoEMan")) {
        encoder.setPosition(3);
      }
      buttonClicked = false;
      currentMenu = "mandatory";
      lcd.clear();
    }
  }

  else if (currentMenu.equals("runningMan")) {
    lcdPrint("Volume:", 0, 0);
    lcdPrint(String(stepsPerHalfBreath), 8, 0, 4);
    lcdPrintSimp("MAN", 13, 0);
    lcdPrintSimp("Rate:", 0, 1);
    lcdPrint(String(breathsPerMinute), 6, 1, 2);
    lcdPrintSimp("1:" + String(ItoE).substring(0, 1), 10, 1);
    if (!buttonUseState) {
      submenu = encoder.getPosition();
      if (submenu > 3) {
        submenu = 0;
        encoder.setPosition(0);
      }
      else if (submenu == -1) {
        submenu = 3;
      }
      else if (submenu == -2) {
        submenu = 2;
      }
      else if (submenu == -3) {
        submenu = 1;
      }
      else if (submenu < -3) {
        submenu = 0;
        encoder.setPosition(0);
      }
    }
    if (submenu == 0) {
      lcdWrite(byte(5), 15, 1);
      lcdWrite(' ', 7, 0);
      lcdWrite(' ', 5, 1);
      lcdWrite(' ', 9, 1);
      if (buttonClicked) {
        buttonClicked = false;
        encoder.setPosition(4);
        stepperHome("Homing...");
        currentMenu = "mandatory";
        lcd.clear();
      }

    }
    else if (submenu == 1) {
      //volume
      lcdWrite(' ', 15, 1);
      lcdWrite((buttonUseState) ? byte(1) : byte(0), 7, 0);
      lcdWrite(' ', 5, 1);
      lcdWrite(' ', 9, 1);
      if (buttonClicked || buttonUseState) {
        if (!buttonUseState) {
          buttonClicked = false;
          encoder.setPosition(stepsPerHalfBreath / 5);
        }
        buttonUseState = true;
        lcdWrite(byte(1), 7, 0);
        stepsPerHalfBreath = encoder.getPosition() * 5;
        if (stepsPerHalfBreath < 20) {
          stepsPerHalfBreath = 20;
          encoder.setPosition(4);
        }
        if (stepsPerHalfBreath > 130) {
          stepsPerHalfBreath = 130;
          encoder.setPosition(26);
        }
        lcdPrint(String(stepsPerHalfBreath), 8, 0, 4);
        if (buttonClicked) {
          buttonClicked = false;
          buttonUseState = false;
          encoder.setPosition(1);
        }
      }
    }
    else if (submenu == 2) {
      //rate
      lcdWrite(' ', 15, 1);
      lcdWrite(' ', 7, 0);
      lcdWrite((buttonUseState) ? byte(1) : byte(0), 5, 1);
      lcdWrite(' ', 9, 1);
      if (buttonClicked || buttonUseState) {
        if (!buttonUseState) {
          buttonClicked = false;
          encoder.setPosition(breathsPerMinute / 2);
        }
        buttonUseState = true;
        lcdWrite(byte(1), 5, 1);
        breathsPerMinute = encoder.getPosition() * 2;
        if (breathsPerMinute < 6) {
          breathsPerMinute = 6;
          encoder.setPosition(3);
        }
        if (breathsPerMinute > 60) {
          breathsPerMinute = 60;
          encoder.setPosition(30);
        }
        lcdPrint(String(breathsPerMinute), 6, 1, 2);
        if (buttonClicked) {
          buttonClicked = false;
          buttonUseState = false;
          encoder.setPosition(2);
        }
      }
    }
    else if (submenu == 3) {
      //i:e
      lcdWrite(' ', 15, 1);
      lcdWrite(' ', 7, 0);
      lcdWrite(' ', 5, 1);
      lcdWrite((buttonUseState) ? byte(1) : byte(0), 9, 1);
      if (buttonClicked || buttonUseState) {
        if (!buttonUseState) {
          buttonClicked = false;
          encoder.setPosition(ItoE);
        }
        buttonUseState = true;
        lcdWrite(byte(1), 9, 1);
        ItoE = encoder.getPosition();
        if (ItoE < 1) {
          ItoE = 1;
          encoder.setPosition(1);
        }
        if (ItoE > 4) {
          ItoE = 4;
          encoder.setPosition(4);
        }
        lcdPrintSimp("1:" + String(ItoE).substring(0, 1), 10, 1);
        if (buttonClicked) {
          buttonClicked = false;
          buttonUseState = false;
          encoder.setPosition(3);
        }
      }
    }

    ///////////////////RUN MAND////////////////////////////////////////
    //Serial.println(String(stepsPerHalfBreath) + " " + String(breathsPerMinute)  + " " + String(ItoE));
    mandatoryMode();
  }


  else if (currentMenu.equals("responsive")) {
    rowsInMenu = 5;
    if (selectedRow == 1) {
      if (!cleared) {
        lcd.clear();
        cleared = true;
      }
      lcdWrite((selectedRow == 1) ? byte(1) : byte(0), 0, 0);
      lcdPrintSimp("Volume: " + String(stepsPerHalfBreath), 1, 0);
      lcdWrite((selectedRow == 2) ? byte(1) : byte(0), 0, 1);
      lcdPrintSimp("PEEP: " + String(peep), 1, 1);
      lcdWrite(byte(2), 15, 1);
    }
    else if (selectedRow == 2) {
      if (!cleared) {
        lcd.clear();
        cleared = true;
      }
      lcdWrite((selectedRow == 2) ? byte(1) : byte(0), 0, 0);
      lcdPrintSimp("PEEP: " + String(peep), 1, 0);
      lcdWrite((selectedRow == 3) ? byte(1) : byte(0), 0, 1);
      lcdPrintSimp("I/E: 1:" + String(ItoE).substring(0, 1), 1, 1);
      lcdWrite(byte(4), 15, 1);
    }
    else if (selectedRow == 3) {
      if (!cleared) {
        lcd.clear();
        cleared = true;
      }
      lcdWrite((selectedRow == 3) ? byte(1) : byte(0), 0, 0);
      lcdPrintSimp("I/E: 1:" + String(ItoE).substring(0, 1), 1, 0);
      lcdWrite((selectedRow == 4) ? byte(1) : byte(0), 0, 1);
      lcdPrintSimp("START", 1, 1);
      lcdWrite(byte(4), 15, 1);
    }
    else if (selectedRow == 4 || selectedRow == 5) {
      if (!cleared) {
        lcd.clear();
        cleared = true;
      }
      lcdWrite((selectedRow == 4) ? byte(1) : byte(0), 0, 0);
      lcdPrintSimp("START", 1, 0);
      lcdWrite((selectedRow == 5) ? byte(1) : byte(0), 0, 1);
      lcdPrintSimp("BACK", 1, 1);
      lcdWrite(byte(3), 15, 1);
    }
    if (buttonClicked && selectedRow == 1) {
      buttonClicked = false;
      encoder.setPosition(stepsPerHalfBreath / 5);
      currentMenu = "volumeRes";
    }
    else if (buttonClicked && selectedRow == 2) {
      buttonClicked = false;
      encoder.setPosition(peep);
      currentMenu = "peepRes";
    }
    else if (buttonClicked && selectedRow == 3) {
      buttonClicked = false;
      encoder.setPosition(ItoE);
      currentMenu = "ItoEMan";
    }
    else if (buttonClicked && selectedRow == 4) {
      buttonClicked = false;
      encoder.setPosition(0);
      breathsPerMinute = resStartBPM;
      currentMenu = "runningRes";
      lcd.clear();
    }
    else if (buttonClicked && selectedRow == 5) {
      buttonClicked = false;
      encoder.setPosition(2);
      currentMenu = "mode";
      lcd.clear();
    }
  }

  else if (currentMenu.equals("volumeRes")
           || currentMenu.equals("peepRes")
           || currentMenu.equals("ItoERes")) {
    lcdWrite(' ', 0, 0);
    lcdWrite(' ', 0, 1);
    if (currentMenu.equals("volumeRes")) {
      lcdWrite(byte(1), 8, 0);
      stepsPerHalfBreath = encoder.getPosition() * 5;
      if (stepsPerHalfBreath < 20) {
        stepsPerHalfBreath = 20;
        encoder.setPosition(4);
      }
      if (stepsPerHalfBreath > 130) {
        stepsPerHalfBreath = 130;
        encoder.setPosition(26);
      }
      lcdPrint(String(stepsPerHalfBreath), 9, 0, 4);
    }
    else if (currentMenu.equals("peepRes")) {
      lcdWrite(byte(1), 6, 0);
      peep = encoder.getPosition();
      if (peep < 0) {
        peep = 0;
        encoder.setPosition(0);
      }
      if (peep > 15) {
        peep = 15;
        encoder.setPosition(15);
      }
      lcdPrint(String(peep), 7, 0, 2);
    }
    else if (currentMenu.equals("ItoERes")) {
      lcdWrite(byte(1), 5, 0);
      ItoE = encoder.getPosition();
      if (ItoE < 1) {
        ItoE = 1;
        encoder.setPosition(1);
      }
      if (ItoE > 4) {
        ItoE = 4;
        encoder.setPosition(4);
      }
      lcdPrint(String(ItoE).substring(0, 1), 8, 0);
    }
    if (buttonClicked) {
      if (currentMenu.equals("volumeRes")) {
        encoder.setPosition(1);
      }
      else if (currentMenu.equals("peepRes")) {
        encoder.setPosition(2);
      }
      else if (currentMenu.equals("ItoERes")) {
        encoder.setPosition(3);
      }
      buttonClicked = false;
      currentMenu = "responsive";
      lcd.clear();
    }
  }

  else if (currentMenu.equals("runningRes")) {
    //Serial.println("res");
    lcdPrint("Volume:", 0, 0);
    lcdPrint(String(stepsPerHalfBreath), 8, 0, 4);
    lcdPrintSimp("RES", 13, 0);
    lcdPrintSimp("PEEP:", 0, 1);
    lcdPrint(String(peep), 6, 1, 2);
    lcdPrintSimp("1:" + String(ItoE).substring(0, 1), 10, 1);
    if (!buttonUseState) {
      submenu = encoder.getPosition();
      if (submenu > 3) {
        submenu = 0;
        encoder.setPosition(0);
      }
      else if (submenu == -1) {
        submenu = 3;
      }
      else if (submenu == -2) {
        submenu = 2;
      }
      else if (submenu == -3) {
        submenu = 1;
      }
      else if (submenu < -3) {
        submenu = 0;
        encoder.setPosition(0);
      }
    }
    if (submenu == 0) {
      lcdWrite(byte(5), 15, 1);
      lcdWrite(' ', 7, 0);
      lcdWrite(' ', 5, 1);
      lcdWrite(' ', 9, 1);
      if (buttonClicked) {
        buttonClicked = false;
        encoder.setPosition(4);
        stepperHome("Homing...");
        currentMenu = "responsive";
        lcd.clear();
      }

    }
    else if (submenu == 1) {
      //volume
      lcdWrite(' ', 15, 1);
      lcdWrite((buttonUseState) ? byte(1) : byte(0), 7, 0);
      lcdWrite(' ', 5, 1);
      lcdWrite(' ', 9, 1);
      if (buttonClicked || buttonUseState) {
        if (!buttonUseState) {
          buttonClicked = false;
          encoder.setPosition(stepsPerHalfBreath / 5);
        }
        buttonUseState = true;
        lcdWrite(byte(1), 7, 0);
        stepsPerHalfBreath = encoder.getPosition() * 5;
        if (stepsPerHalfBreath < 20) {
          stepsPerHalfBreath = 20;
          encoder.setPosition(4);
        }
        if (stepsPerHalfBreath > 130) {
          stepsPerHalfBreath = 130;
          encoder.setPosition(26);
        }
        lcdPrint(String(stepsPerHalfBreath), 8, 0, 4);
        if (buttonClicked) {
          buttonClicked = false;
          buttonUseState = false;
          encoder.setPosition(1);
        }
      }
    }
    else if (submenu == 2) {
      //rate
      lcdWrite(' ', 15, 1);
      lcdWrite(' ', 7, 0);
      lcdWrite((buttonUseState) ? byte(1) : byte(0), 5, 1);
      lcdWrite(' ', 9, 1);
      if (buttonClicked || buttonUseState) {
        if (!buttonUseState) {
          buttonClicked = false;
          encoder.setPosition(peep);
        }
        buttonUseState = true;
        lcdWrite(byte(1), 5, 1);
        peep = encoder.getPosition();
        if (peep < 0) {
          peep = 0;
          encoder.setPosition(0);
        }
        if (peep > 15) {
          peep = 15;
          encoder.setPosition(15);
        }
        lcdPrint(String(peep), 6, 1, 2);
        if (buttonClicked) {
          buttonClicked = false;
          buttonUseState = false;
          encoder.setPosition(2);
        }
      }
    }
    else if (submenu == 3) {
      //i:e
      lcdWrite(' ', 15, 1);
      lcdWrite(' ', 7, 0);
      lcdWrite(' ', 5, 1);
      lcdWrite((buttonUseState) ? byte(1) : byte(0), 9, 1);
      if (buttonClicked || buttonUseState) {
        if (!buttonUseState) {
          buttonClicked = false;
          encoder.setPosition(ItoE);
        }
        buttonUseState = true;
        lcdWrite(byte(1), 9, 1);
        ItoE = encoder.getPosition();
        if (ItoE < 1) {
          ItoE = 1;
          encoder.setPosition(1);
        }
        if (ItoE > 4) {
          ItoE = 4;
          encoder.setPosition(4);
        }
        lcdPrintSimp("1:" + String(ItoE).substring(0, 1), 10, 1);
        if (buttonClicked) {
          buttonClicked = false;
          buttonUseState = false;
          encoder.setPosition(3);
        }
      }
    }

    ///////////////////RUN RESP//////////////////////////
    //Serial.println(String(stepsPerHalfBreath) + " " + String(peep)  + " " + String(ItoE));
    responsiveMode();
  }

  else if (currentMenu.equals("settings")) {
    //Serial.println(encoder.getPosition());
    rowsInMenu = 3;
    if (selectedRow == 1) {
      if (!cleared) {
        lcd.clear();
        cleared = true;
      }
      lcdWrite((selectedRow == 1) ? byte(1) : byte(0), 0, 0);
      lcdPrintSimp("Pres warn: ", 1, 0);
      lcdPrint(String(pressureWarning, 1), 12, 0, 3);
      lcdWrite((selectedRow == 2) ? byte(1) : byte(0), 0, 1);
      lcdPrintSimp("Resp BPM: ", 1, 1);
      lcdPrint(String(resStartBPM), 11, 1, 2);
      lcdWrite(byte(2), 15, 1);
    }
    else if (selectedRow == 2 || selectedRow == 3) {
      if (!cleared) {
        lcd.clear();
        cleared = true;
      }
      lcdWrite((selectedRow == 2) ? byte(1) : byte(0), 0, 0);
      lcdPrintSimp("Resp BPM:", 1, 0);
      lcdPrint(String(resStartBPM), 11, 0, 2);
      lcdWrite((selectedRow == 3) ? byte(1) : byte(0), 0, 1);
      lcdPrintSimp("BACK", 1, 1);
      lcdWrite(byte(3), 15, 1);
    }
    if (buttonClicked && selectedRow == 1) {
      buttonClicked = false;
      currentMenu = "presSet";
      encoder.setPosition(pressureWarning / .1);
    }
    else if (buttonClicked && selectedRow == 2) {
      buttonClicked = false;
      currentMenu = "BPMSet";
      encoder.setPosition(resStartBPM / 2);
    }
    else if (buttonClicked && selectedRow == 3) {
      buttonClicked = false;
      currentMenu = "mode";
      lcd.clear();
      encoder.setPosition( 3);
    }
  }
  else if (currentMenu.equals("presSet")
           || currentMenu.equals("BPMSet")) {
    lcdWrite(' ', 0, 0);
    lcdWrite(' ', 0, 1);
    if (currentMenu.equals("presSet")) {
      lcdWrite(byte(1), 11, 0);
      pressureWarning = encoder.getPosition() * .1;
      if (pressureWarning < 2) {
        pressureWarning = 2;
        encoder.setPosition(20);
      }
      if (pressureWarning > 5) {
        pressureWarning = 5;
        encoder.setPosition(50);
      }
      lcdPrint(String(pressureWarning, 1), 12, 0, 3);
    }
    else if (currentMenu.equals("BPMSet")) {
      lcdWrite(byte(1), 10, 0);
      resStartBPM = encoder.getPosition() * 2;
      if (resStartBPM < 6) {
        resStartBPM = 6;
        encoder.setPosition(3);
      }
      if (resStartBPM > 60) {
        resStartBPM = 60;
        encoder.setPosition(30);
      }
      lcdPrint(String(resStartBPM), 11, 0, 2);
    }
    if (buttonClicked) {
      if (currentMenu.equals("presSet")) {
        encoder.setPosition(1);
      }
      else if (currentMenu.equals("BPMSet")) {
        encoder.setPosition(2);
      }
      buttonClicked = false;
      currentMenu = "settings";
      lcd.clear();
    }
  }


  lastButtonState = buttonRead;
  prevSelectedRow = selectedRow;
  prevSubmenu = submenu;

  Serial.println(String(micros() - lastTime) + "\t" + String(speed));
  lastTime = micros();
}

ISR(TIMER1_COMPA_vect) {
  OCR1A = compare_time;
  halfStepsElapsed++;
}