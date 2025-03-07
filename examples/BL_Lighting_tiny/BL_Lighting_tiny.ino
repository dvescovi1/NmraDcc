

#include <NmraDcc.h>
#include <EEPROM.h>

// Define the Arduino input Pin number for the DCC Signal 
#define DCC_PIN           2         // PB2 int0
#define LEARN_BUTTON_PIN  1


#define LED1             0
#define LED2_GREEN       3
#define LED2_RED         4

#define EEPROM_VALID      0xa5

NmraDcc  Dcc ;

typedef enum
{
  DOOR_OFF,
  DOOR_GREEN,
  DOOR_RED,
  DOOR_LAST
} DOOR_LED;

struct MyEEPROM {
  byte valid;
  uint16_t MyAddr;
  DCC_ADDR_TYPE MyAddrType;
};

MyEEPROM ee_addr;

DOOR_LED door_state = DOOR_OFF;
DOOR_LED previous_door_state = DOOR_OFF;

int buttonState = 0;        // Variable for reading the button status
int lastButtonState = 0;    // Variable to store the last button state
unsigned long pressStartTime = 0; // Time when button was pressed
unsigned long pressDuration = 0;  // Duration of button press

unsigned long previousFlashLED1Millis = 0;
unsigned long previousFlashLED2Millis = 0;
int flashRate = 500;

bool learning_mode = false;
bool flashing_LED2_mode = false;
bool last_FN_X_state;
bool last_FN_Y_state;
bool last_FN_Z_state;

void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
  boolean FN_X_state;
  boolean FN_Y_state;
  boolean FN_Z_state;

  if ( FuncGrp ==  FN_13_20)
   {
      FN_X_state = FuncState & FN_BIT_13;
      FN_Y_state = FuncState & FN_BIT_14;
      FN_Z_state = FuncState & FN_BIT_15;
      if (learning_mode && (!last_FN_X_state && FN_X_state))         // off to on in learning state
      {
        ee_addr.valid = EEPROM_VALID;
        ee_addr.MyAddrType = AddrType;
        ee_addr.MyAddr = Addr;
        EEPROM.put(0, ee_addr);
        learning_mode = false;
      }
      else if ((ee_addr.MyAddrType == AddrType) && (ee_addr.MyAddr == Addr))
      {
        // led1
        if (digitalRead(LED1) == LOW && FN_X_state)
        {
          digitalWrite(LED1, HIGH);   // turn on
        }          
        if (digitalRead(LED1) == HIGH && !FN_X_state)
        {
          digitalWrite(LED1, LOW);   // turn off
        }
        
        // flash LED2
        flashing_LED2_mode = FN_Z_state;
        if (last_FN_Z_state && !FN_Z_state)   // on to off
        {
          door_state = DOOR_OFF;              // also reset LED2 state machine
          previous_door_state = DOOR_OFF;
          if (FN_Y_state)
            last_FN_Y_state = false;
        }

        // updating LED2 actually takes place in loop
        // led2
        if (!last_FN_Y_state && FN_Y_state)   // off to on
        {
          switch (previous_door_state)
          {
            case DOOR_OFF:
              door_state = DOOR_GREEN;
              break;
            case DOOR_GREEN:
              door_state = DOOR_RED;
              break;
            case DOOR_RED:
              door_state = DOOR_GREEN;
              break;
            default:
              door_state = DOOR_OFF;
              break;
          }
          previous_door_state = door_state;
        }
        if (last_FN_Y_state && !FN_Y_state)   // on to off
        {
          door_state = DOOR_OFF;
        }

      }
      last_FN_X_state = FN_X_state;
      last_FN_Y_state = FN_Y_state;
      last_FN_Z_state = FN_Z_state;
    }
}

void setup()
{
  // leds
  digitalWrite(LED1, LOW);   // turn the LED1 off
  pinMode(LED1, OUTPUT);
  digitalWrite(LED2_GREEN, LOW);   // turn the GREEN off
  pinMode(LED2_GREEN, OUTPUT);
  digitalWrite(LED2_RED, LOW);     // turn the RED off
  pinMode(LED2_RED, OUTPUT);

  EEPROM.get(0, ee_addr);
  if (ee_addr.valid != EEPROM_VALID)
  {
    learning_mode = true;
  }

  // button
  pinMode(LEARN_BUTTON_PIN, INPUT_PULLUP);  // Set the button pin as input with internal pull-up resistor
 
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  // Many Arduino Cores now support the digitalPinToInterrupt() function that makes it easier to figure out the
  // Interrupt Number for the Arduino Pin number, which reduces confusion. 
#ifdef digitalPinToInterrupt
  Dcc.pin(DCC_PIN, 0);
#else
  Dcc.pin(0, DCC_PIN, 1);
#endif
  
  Dcc.init( MAN_ID_DIY, 10, 0, 0 );
}

void loop()
{
  unsigned long currentFlashLED1Millis;
  unsigned long currentFlashLED2Millis;


  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

  buttonState = digitalRead(LEARN_BUTTON_PIN);  // Read the current state of the button

  // Check if the button state has changed
  if (buttonState != lastButtonState) {
    if (buttonState == LOW) {
      // Button was pressed, record the start time
      pressStartTime = millis();
    } else {
      // Button was released, calculate the duration
      pressDuration = millis() - pressStartTime;

      if (pressDuration > 0)
      {
        learning_mode = !learning_mode;
      }
      pressDuration = 0;
    }
    lastButtonState = buttonState;  // Update the last button state
  }

  currentFlashLED1Millis = millis(); // Get the current time

  if (learning_mode)
  {
    flashing_LED2_mode = false;
    door_state = DOOR_OFF;
    previous_door_state = DOOR_OFF;
    if (currentFlashLED1Millis - previousFlashLED1Millis >= flashRate) {
      previousFlashLED1Millis = currentFlashLED1Millis; // Save the current time
      
      // Toggle the LED1
      if (digitalRead(LED1) == LOW) {
        digitalWrite(LED1, HIGH); // Turn the LED on
      } else {
        digitalWrite(LED1, LOW);  // Turn the LED off
      }
    }
  }

  currentFlashLED2Millis = millis(); // Get the current time

  if (flashing_LED2_mode)
  {
    if (currentFlashLED2Millis - previousFlashLED2Millis >= flashRate) {
      previousFlashLED2Millis = currentFlashLED2Millis; // Save the current time
      
      // Toggle the active LED2 state
      switch (door_state)
      {
        case DOOR_OFF:
          digitalWrite(LED2_GREEN, LOW);
          digitalWrite(LED2_RED, LOW);
          break;
        case DOOR_GREEN:
          if (digitalRead(LED2_GREEN) == LOW) {
            digitalWrite(LED2_GREEN, HIGH); // Turn the LED on
          } else {
            digitalWrite(LED2_GREEN, LOW);  // Turn the LED off
          }
          break;
        case DOOR_RED:
          if (digitalRead(LED2_RED) == LOW) {
            digitalWrite(LED2_RED, HIGH); // Turn the LED on
          } else {
            digitalWrite(LED2_RED, LOW);  // Turn the LED off
          }
          break;
        default:
            digitalWrite(LED2_GREEN, LOW);
            digitalWrite(LED2_RED, LOW);
            break;
      }
   }
  }
  else
  {
    switch (door_state)
    {
      case DOOR_OFF:
        digitalWrite(LED2_GREEN, LOW);
        digitalWrite(LED2_RED, LOW);
        break;
      case DOOR_GREEN:
        digitalWrite(LED2_GREEN, HIGH);
        digitalWrite(LED2_RED, LOW);
        break;
      case DOOR_RED:
        digitalWrite(LED2_GREEN, LOW);
        digitalWrite(LED2_RED, HIGH);
        break;
      default:
        digitalWrite(LED2_GREEN, LOW);
        digitalWrite(LED2_RED, LOW);
        break;
    }
  }
}
