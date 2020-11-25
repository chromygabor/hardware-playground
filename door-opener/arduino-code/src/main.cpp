#include <Arduino.h>

#define SW_DOOR_2_CLOSE 12
#define SW_DOOR_2_OPEN 11
#define SW_DOOR_1_CLOSE 10
#define SW_DOOR_1_OPEN 9
#define SW_LIGHT_SENSOR 8
#define SW_SEPARATE_2 7
#define BT_TOGGLE 6

#define OUT_MOT_1_OPEN 3
#define OUT_MOT_1_CLOSE 2
#define OUT_MOT_2_OPEN 5
#define OUT_MOT_2_CLOSE 4

#define CONST_MOT_ON 1
#define CONST_MOT_OFF 0

#define CONST_INPUT_CLOSE 1
#define CONST_INPUT_OPENED 0

#define MOT_TRESHOLD 5000

uint8_t is_DOOR_1_OPENED = CONST_INPUT_OPENED;
uint8_t is_DOOR_1_CLOSED = CONST_INPUT_OPENED;
uint8_t is_DOOR_2_OPENED = CONST_INPUT_OPENED;
uint8_t is_DOOR_2_CLOSED = CONST_INPUT_OPENED;
uint8_t is_DARK = CONST_INPUT_OPENED;
uint8_t is_DOOR_2_SEPARATED = CONST_INPUT_OPENED;
uint8_t is_BTN_TOGGLE_PRESSED = CONST_INPUT_OPENED;

uint8_t MASK_DOOR_1_OPENED = 0x0001;
uint8_t MASK_DOOR_1_CLOSED = 0x0002;
uint8_t MASK_DOOR_2_OPENED = 0x0004;
uint8_t MASK_DOOR_2_CLOSED = 0x0008;
uint8_t MASK_DARK = 0x0010;
uint8_t MASK_DOOR_2_SEPARATED = 0x0020;
uint8_t MASK_BTN_TOGGLE_PRESSED = 0x0040;


uint8_t is_MOT_1_ON = CONST_MOT_OFF;
long time_MOT_1_STARTED = 0;
uint8_t is_MOT_2_ON = CONST_MOT_OFF;
long time_MOT_2_STARTED = 0;

unsigned long loopCounter = 0;
unsigned int lastValue = 0;

unsigned int valueProposal = 0;
unsigned long valueProposalAt = 0;

void setup()
{
  pinMode(OUT_MOT_1_OPEN, OUTPUT);
  pinMode(OUT_MOT_1_CLOSE, OUTPUT);
  pinMode(OUT_MOT_2_OPEN, OUTPUT);
  pinMode(OUT_MOT_2_CLOSE, OUTPUT);

  digitalWrite(OUT_MOT_1_OPEN, CONST_MOT_OFF);
  digitalWrite(OUT_MOT_1_CLOSE, CONST_MOT_OFF);
  digitalWrite(OUT_MOT_2_OPEN, CONST_MOT_OFF);
  digitalWrite(OUT_MOT_2_CLOSE, CONST_MOT_OFF);

  Serial.begin(9600);
  // initialize LED digital pin as an output.
  pinMode(SW_DOOR_1_OPEN, INPUT_PULLUP);
  pinMode(SW_DOOR_1_CLOSE, INPUT_PULLUP);
  pinMode(SW_DOOR_2_OPEN, INPUT_PULLUP);
  pinMode(SW_DOOR_2_CLOSE, INPUT_PULLUP);
  pinMode(SW_LIGHT_SENSOR, INPUT_PULLUP);
  pinMode(BT_TOGGLE, INPUT_PULLUP);
  pinMode(SW_SEPARATE_2, INPUT_PULLUP);

  is_DOOR_1_OPENED = !digitalRead(SW_DOOR_1_OPEN);
  is_DOOR_1_CLOSED = !digitalRead(SW_DOOR_1_CLOSE);
  is_DOOR_2_OPENED = !digitalRead(SW_DOOR_2_OPEN);
  is_DOOR_2_CLOSED = !digitalRead(SW_DOOR_2_CLOSE);

  if( (!is_DOOR_1_OPENED && !is_DOOR_1_CLOSED) ||
    (!is_DOOR_2_OPENED && !is_DOOR_2_CLOSED)) {
    delay(500);
    if( !is_DOOR_1_OPENED && !is_DOOR_1_CLOSED ) {
      Serial.println("Door 1 is on halfway, closing");
      digitalWrite(OUT_MOT_1_CLOSE, CONST_MOT_ON);
      time_MOT_1_STARTED = millis();
    }

    if( !is_DOOR_2_OPENED && !is_DOOR_2_CLOSED ) {
      Serial.println("Door 2 is on halfway, closing");
      digitalWrite(OUT_MOT_2_CLOSE, CONST_MOT_ON);
      time_MOT_2_STARTED = millis();
    }
  }

  Serial.println("Init completed");
}


void openMotors(
  uint8_t is_DOOR_1_OPENED,
  uint8_t is_DOOR_1_CLOSED,
  uint8_t is_DOOR_2_OPENED,
  uint8_t is_DOOR_2_CLOSED
) {
  Serial.println("Open motors");
  digitalWrite(OUT_MOT_1_CLOSE, CONST_MOT_OFF);
  digitalWrite(OUT_MOT_2_CLOSE, CONST_MOT_OFF);

  if(!is_DOOR_1_OPENED) {
    digitalWrite(OUT_MOT_1_OPEN, CONST_MOT_ON);
    time_MOT_1_STARTED = millis();
  }
  if(!is_DOOR_2_SEPARATED && !is_DOOR_2_OPENED) {
    digitalWrite(OUT_MOT_2_OPEN, CONST_MOT_ON);
    time_MOT_2_STARTED = millis();
  }
}

void closeMotors(
  uint8_t is_DOOR_1_OPENED,
  uint8_t is_DOOR_1_CLOSED,
  uint8_t is_DOOR_2_OPENED,
  uint8_t is_DOOR_2_CLOSED
) {
  Serial.println("Close motors");
  digitalWrite(OUT_MOT_1_OPEN, CONST_MOT_OFF);
  digitalWrite(OUT_MOT_2_OPEN, CONST_MOT_OFF);
  
  if(!is_DOOR_1_CLOSED) {
    digitalWrite(OUT_MOT_1_CLOSE, CONST_MOT_ON);
    time_MOT_1_STARTED = millis();
  }
  if(!is_DOOR_2_SEPARATED && !is_DOOR_2_CLOSED) {
    digitalWrite(OUT_MOT_2_CLOSE, CONST_MOT_ON);
    time_MOT_2_STARTED = millis();
  }
}


void loop()
{
  loopCounter = loopCounter + 1;
  
  is_DOOR_1_OPENED = !digitalRead(SW_DOOR_1_OPEN);
  is_DOOR_1_CLOSED = !digitalRead(SW_DOOR_1_CLOSE);
  is_DOOR_2_OPENED = !digitalRead(SW_DOOR_2_OPEN);
  is_DOOR_2_CLOSED = !digitalRead(SW_DOOR_2_CLOSE);
  is_DARK = !digitalRead(SW_LIGHT_SENSOR);
  is_BTN_TOGGLE_PRESSED = !digitalRead(BT_TOGGLE);
  is_MOT_1_ON = (digitalRead(OUT_MOT_1_OPEN) == CONST_MOT_ON) || (digitalRead(OUT_MOT_1_CLOSE) == CONST_MOT_ON);
  is_MOT_2_ON = (digitalRead(OUT_MOT_2_OPEN) == CONST_MOT_ON) || (digitalRead(OUT_MOT_2_CLOSE) == CONST_MOT_ON);


  is_DOOR_2_SEPARATED = !digitalRead(SW_SEPARATE_2);

  long now = millis();

  if(is_MOT_1_ON && (now - time_MOT_1_STARTED) > MOT_TRESHOLD ) {
    Serial.println("Mot1 threshold reached");
    digitalWrite(OUT_MOT_1_OPEN, CONST_MOT_OFF);
    digitalWrite(OUT_MOT_1_CLOSE, CONST_MOT_OFF);
  }

  if(is_MOT_2_ON && (now - time_MOT_2_STARTED) > MOT_TRESHOLD ) {
    Serial.println("Mot2 threshold reached");
    digitalWrite(OUT_MOT_2_OPEN, CONST_MOT_OFF);
    digitalWrite(OUT_MOT_2_CLOSE, CONST_MOT_OFF);
  }

  unsigned int value = 
    (is_DOOR_1_OPENED * MASK_DOOR_1_OPENED)
    | (is_DOOR_1_CLOSED * MASK_DOOR_1_CLOSED)
    | (is_DOOR_2_OPENED * MASK_DOOR_2_OPENED)
    | (is_DOOR_2_CLOSED * MASK_DOOR_2_CLOSED)
    | (is_DARK * MASK_DARK)
    | (is_BTN_TOGGLE_PRESSED * MASK_BTN_TOGGLE_PRESSED)
    | (is_DOOR_2_SEPARATED * MASK_DOOR_2_SEPARATED)
  ;

  if( (valueProposal != value) ) {
    valueProposalAt = loopCounter;
    valueProposal = value;
  }

  if((lastValue != valueProposal) && (loopCounter == valueProposalAt + 1000)) {
    Serial.print("Changed: ");
    Serial.println(value);
    unsigned long int lv = lastValue;
    lastValue = valueProposal;

    if(!(lv & MASK_BTN_TOGGLE_PRESSED) && (lastValue & MASK_BTN_TOGGLE_PRESSED)) {
      if(is_DOOR_1_OPENED == CONST_INPUT_CLOSE) {
        Serial.println("Forcing to close");
        // start motors to close
        closeMotors(is_DOOR_1_OPENED, is_DOOR_1_CLOSED, is_DOOR_2_OPENED, is_DOOR_2_CLOSED);
      } else if(is_DOOR_1_CLOSED == CONST_INPUT_CLOSE) {
        Serial.println("Forcing to open");
        // start motors to open
        openMotors(is_DOOR_1_OPENED, is_DOOR_1_CLOSED, is_DOOR_2_OPENED, is_DOOR_2_CLOSED);
      } else {
        if(OUT_MOT_1_OPEN) {
          Serial.println("Changing direction, closing");
          closeMotors(is_DOOR_1_OPENED, is_DOOR_1_CLOSED, is_DOOR_2_OPENED, is_DOOR_2_CLOSED);
        } else {
          Serial.println("Changing direction, opening");
          openMotors(is_DOOR_1_OPENED, is_DOOR_1_CLOSED, is_DOOR_2_OPENED, is_DOOR_2_CLOSED);
        }
        // change motors direction

      }
    }

    if(!(lv & MASK_DARK) && (lastValue & MASK_DARK)) {
      Serial.println("Getting dark");
      // start motors to close
      closeMotors(is_DOOR_1_OPENED, is_DOOR_1_CLOSED, is_DOOR_2_OPENED, is_DOOR_2_CLOSED);
    } else if((lv & MASK_DARK) && !(lastValue & MASK_DARK)) {
      Serial.println("Getting dawn");
      // start motors to open
      openMotors(is_DOOR_1_OPENED, is_DOOR_1_CLOSED, is_DOOR_2_OPENED, is_DOOR_2_CLOSED);
    } 

    //####### Door1
    if(!(lv & MASK_DOOR_1_CLOSED) && (lastValue & MASK_DOOR_1_CLOSED)) {
      Serial.println("DOOR1 is closed");
      //Motor1 stop
      Serial.println("Stop motor 1");

      digitalWrite(OUT_MOT_1_OPEN, CONST_MOT_OFF);
      digitalWrite(OUT_MOT_1_CLOSE, CONST_MOT_OFF);
    }
    if((lv & MASK_DOOR_1_CLOSED) && !(lastValue & MASK_DOOR_1_CLOSED)) {
      Serial.println("DOOR1 is opening");
    }
    if(!(lv & MASK_DOOR_1_OPENED) && (lastValue & MASK_DOOR_1_OPENED)) {
      Serial.println("DOOR1 is opened");
      //Motor1 stop
      Serial.println("Stop motor 1");

      digitalWrite(OUT_MOT_1_OPEN, CONST_MOT_OFF);
      digitalWrite(OUT_MOT_1_CLOSE, CONST_MOT_OFF);
    }
    if((lv & MASK_DOOR_1_OPENED) && !(lastValue & MASK_DOOR_1_OPENED)) {
      Serial.println("DOOR1 is closing");
    }

    //####### Door2
    if(!(lv & MASK_DOOR_2_CLOSED) && (lastValue & MASK_DOOR_2_CLOSED)) {
      Serial.println("DOOR2 is closed");
      Serial.println("Stop motor 2");

      //Motor2 stop
      digitalWrite(OUT_MOT_2_OPEN, CONST_MOT_OFF);
      digitalWrite(OUT_MOT_2_CLOSE, CONST_MOT_OFF);
    }
    if((lv & MASK_DOOR_2_CLOSED) && !(lastValue & MASK_DOOR_2_CLOSED)) {
      Serial.println("DOOR2 is opening");
    }
    if(!(lv & MASK_DOOR_2_OPENED) && (lastValue & MASK_DOOR_2_OPENED)) {
      Serial.println("DOOR2 is opened");
      Serial.println("Stop motor 2");
      //Motor2 stop
      digitalWrite(OUT_MOT_2_OPEN, CONST_MOT_OFF);
      digitalWrite(OUT_MOT_2_CLOSE, CONST_MOT_OFF);
    }
    if((lv & MASK_DOOR_2_OPENED) && !(lastValue & MASK_DOOR_2_OPENED)) {
      Serial.println("DOOR2 is closing");
    }


  }
}
