
#include <Wire.h>

//---- DATA TYPES ----
const short MODE_TEMPERATURE = 0;
const short MODE_COUNTDOWN = 1;

const short NO_INPUT = 0;
const short TIME_INPUT_HOURS = 1;
const short TIME_INPUT_MINUTES = 2;
const short TEMP_INPUT = 3;

struct RotateEncoder {
  bool inRotation = false;
  bool clockwise = false;
  bool positionRead = false;
  unsigned long lastRotationTime = 0;
  short position = 0;
};

struct Button {
  short pin;
  bool state = false;
  bool stateRead = false;
  unsigned long lastChange = 0;
};

struct Countdown {
  int hours = 0;
  int minutes = 0;
  bool hoursSet = false;
  bool minutesSet = false;
};

struct Temperature {
  int value = 0;
  bool set = false;
};

struct Display {
  bool update = false;
};

struct State {
  boolean mode = TEMP_INPUT;
  short inputMode = NO_INPUT;
  Temperature temperature = Temperature();
  Countdown countdown = Countdown();
  RotateEncoder *selectWheelPtr;
};

//---- PIN LAYOUT ----

short pin_button = 2;
short pin_clk = 4;
short pin_dt = 3;

short pin_mode_switch = 8;
short pin_buzzer = 9;

/****************************
 * Check for the rotation state of the rotate encoder.
 * Does not decrement below 0
 ****************************/
void checkRotation(RotateEncoder *r) {
  const short MAX_MILLIS = 20;

  // prevent debouncing
  if (r->lastRotationTime + MAX_MILLIS > millis()) {
    return;
  }

  bool current_clk = digitalRead(pin_clk);

  if (r->inRotation == false && current_clk == LOW) {
    r->inRotation = true;
    r->lastRotationTime = 0;

    if (digitalRead(pin_dt) != current_clk) {
      r->clockwise = true;
    } else {
      r->clockwise = false;
    }
  } else if (r->inRotation == true) {
    if (digitalRead(pin_clk) == HIGH && digitalRead(pin_dt) == HIGH) {
      r->position = r->clockwise ? r->position + 1 : (r->position > 0 ? r->position - 1 : r->position);
      r->positionRead = false;
      r->inRotation = false;
      r->clockwise = false;
      r->lastRotationTime = millis();
    }
  }
}

/****************************
 * Check a button state for a given status
 * Includes debouncing prevention
 ****************************/
void checkButton(Button *buttonPtr, bool status) {
  const short MAX_MILLIS = 200;
  short current = digitalRead(buttonPtr->pin);
  
  // prevent debouncing
  if (buttonPtr->lastChange > 0 && buttonPtr->lastChange + MAX_MILLIS > millis()) {
    if(current == buttonPtr->state) {
      buttonPtr->stateRead = false;  
    } else {
      Serial.println("Debounce detected");
      buttonPtr->state = current;      
    }
    
    buttonPtr->lastChange = 0; 
    return;
  }

  if (buttonPtr->state != status) {
    if (current == status) {
      buttonPtr->state = status;
      buttonPtr->lastChange = millis();
    }
  } else {
    if (current != status) {
      buttonPtr->state = !status;
      buttonPtr->lastChange = millis();  
    }
  }
}

/**
 * Check the operation mode based on the mode switch (blue LED).
 */
void checkOperationMode(Button *modeButtonPtr, State *statePtr) {
  checkButton(modeButtonPtr, HIGH);

  if (modeButtonPtr->state == HIGH && modeButtonPtr->stateRead == false && statePtr->mode == MODE_TEMPERATURE) {
    statePtr->mode = MODE_COUNTDOWN;
    statePtr->inputMode = NO_INPUT;
    Serial.println("Switching to mode COUNTDOWN - NO_INPUT");
    modeButtonPtr->stateRead = true;
  } else if (modeButtonPtr->state == LOW && statePtr->mode == MODE_COUNTDOWN) {
    statePtr->mode = MODE_TEMPERATURE;
    statePtr->inputMode = NO_INPUT;
    Serial.println("Switching to mode TEMPERATURE - NO_INPUT");
    modeButtonPtr->stateRead = true;
  }
}

/**
 * Checks the input mode based on the operation mode and the current state of
 * either the temperature or countdown values.1
 */
void checkInputMode(State *statePtr) {
  if (statePtr->mode == MODE_TEMPERATURE) {
    if (statePtr->temperature.set == false) {
      if (statePtr->inputMode == NO_INPUT) {
        statePtr->inputMode = TEMP_INPUT;
        statePtr->selectWheelPtr->position = 0;
        Serial.println("Switching to temp input mode TEMP_INPUT");
      }
    }
  } else {
    if (statePtr->countdown.hoursSet == false && statePtr->inputMode == NO_INPUT) {
      statePtr->inputMode = TIME_INPUT_HOURS;
      statePtr->selectWheelPtr->position = 0;
      Serial.println("Switching to input mode TIME_INPUT_HOURS");
    } else if (statePtr->countdown.hoursSet == true && statePtr->inputMode == TIME_INPUT_HOURS) {
      statePtr->inputMode = TIME_INPUT_MINUTES;
      statePtr->selectWheelPtr->position = 0;
      Serial.println("Switching to input mode TIME_INPUT_MINUTES");
    }
  }
}

/**
 * 
 */
void updateDisplay(Display *displayPtr, State *statePtr) {
  if (displayPtr->update) {
    switch(statePtr->inputMode) {
      case TEMP_INPUT:
        Serial.print("Temperature: ");
        Serial.print(statePtr->selectWheelPtr->position * 10);
        Serial.println("  °C");
        break;
      case TIME_INPUT_HOURS:
        Serial.print("Time: ");
        Serial.print(statePtr->selectWheelPtr->position % 24);
        Serial.print(":");
        Serial.println(statePtr->countdown.minutes);          
        break;
      case TIME_INPUT_MINUTES:
        Serial.print("Time: ");
        Serial.print(statePtr->countdown.hours);            
        Serial.print(":");
        Serial.println(statePtr->selectWheelPtr->position % 60);
        break;
    }          

    Serial.print(statePtr->temperature.value);
    Serial.print(" °C");
    Serial.print(" - ");

    Serial.print(statePtr->countdown.hours);
    Serial.print(":");
    Serial.println(statePtr->countdown.minutes);

    Serial.println();

    displayPtr->update = false;
  }
}

//---- GLOBAL STATE ----
RotateEncoder selectWheel = RotateEncoder();
Button selectButton = Button();
Button modeButton = Button();
Display display = Display();
State state = State();

/********************
   --- SETUP
 ********************/
void setup() {
  pinMode(pin_clk, INPUT_PULLUP);
  pinMode(pin_dt, INPUT_PULLUP);
  pinMode(pin_button, INPUT_PULLUP);

  pinMode(pin_mode_switch, INPUT);
  pinMode(pin_buzzer, OUTPUT);

  Serial.begin(9600);

  selectButton.pin = pin_button;
  modeButton.pin = pin_mode_switch;
  state.selectWheelPtr = &selectWheel;
}

void loop() {
  checkButton(&selectButton, LOW);
  checkRotation(&selectWheel);
  checkOperationMode(&modeButton, &state);
  checkInputMode(&state);


  // ---- Second: Handle the selection wheel ----
  if (selectWheel.positionRead == false) {
    selectWheel.positionRead = true;
    display.update = true;
  }

  //------- First: Check the operation MODE ----
  if (selectButton.state == LOW && selectButton.stateRead == false) {
    switch(state.inputMode) {
      case TEMP_INPUT:
        state.temperature.value = selectWheel.position * 10;
        state.inputMode = NO_INPUT;
        state.temperature.set = true;
        break;
      case TIME_INPUT_HOURS:
        state.countdown.hours = selectWheel.position % 24;
        state.countdown.hoursSet = true;
        break;
      case TIME_INPUT_MINUTES:
        state.countdown.minutes = selectWheel.position % 60;
        state.inputMode = NO_INPUT;
        state.countdown.minutesSet = true;
        break;
    }
   
    selectButton.stateRead = true;
    display.update = true;
  } else if (selectButton.state == HIGH && selectButton.stateRead == false) {
    if (state.mode == MODE_TEMPERATURE && state.temperature.set == true) {
      state.temperature.set = false;
      selectWheel.position = state.temperature.value / 10;
    } else if(state.countdown.minutesSet == true) {
      state.countdown.hoursSet = true;
      state.countdown.minutesSet = true;
      selectWheel.position = state.countdown.hours;
    }

    selectButton.stateRead = true;
    display.update = true;
  }

  updateDisplay(&display, &state);
  //  if (temperature >= 100) {
  //    digitalWrite(pin_buzzer, HIGH);
  //  } else {
  //    digitalWrite(pin_buzzer, LOW);
  //  }
}
