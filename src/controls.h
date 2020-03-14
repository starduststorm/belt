#ifndef CONTROLS_H
#define CONTROLS_H

#define THUMBDIAL1_PIN A8
#define THUMBDIAL2_PIN A2
#define BUTTON_PIN 0

#define THUMBDIAL_THRESHOLD 40

#include <FastLED.h>

class Controls {
private:
  long buttonDownTime = -1;
  long buttonUpTime = -1;
  long singlePressTime = -1;
  bool waitForButtonUp = false;
  
  void (*singlePressHandler)(void);
  void (*doublePressHandler)(void);
  void (*longPressHandler)(void);
  void (*doubleLongPressHandler)(void);

  void (*thumbdial1Handler)(int);
  void (*thumbdial2Handler)(int);

  void handleHandler(void (*handler)(void)) {
    if (handler) {
      (*handler)();
    }
  }

  template<typename T>
  void handleHandler(void (*handler)(T), T arg) {
    if (handler) {
      (*handler)(arg);
    }
  }  

  // FIXME: generalize
  int lastThumbdial1 = -1;
  int lastThumbdial2 = -1;

  unsigned long lastThumbdial1Change = -1;
  unsigned long lastThumbdial2Change = -1;
  
public:
  long longPressInterval = 1000;
  long doublePressInterval = 400;

  Controls() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
  }

  void update() {
    // TODO: generalize all this for different hardware layouts

    int thumbdial1 = analogRead(THUMBDIAL1_PIN);
    // potentiometer reads are noisy, jitter is around ±30 or so. 
    // Wait for significant change to notify the handler, but then still allow smooth updates as the pot is turned
    bool significantChange = abs(lastThumbdial1 - thumbdial1) > THUMBDIAL_THRESHOLD;
    bool recentSignificantChange = (millis() - lastThumbdial1Change < 500);
    bool endpointsChange = lastThumbdial1 != thumbdial1 && (thumbdial1 == 0 || thumbdial1 == 1023);
    
    if (significantChange || recentSignificantChange || endpointsChange) {
      if (significantChange || endpointsChange) {
        lastThumbdial1Change = millis();
        lastThumbdial1 = thumbdial1;
      }
      handleHandler(thumbdial1Handler, thumbdial1);
    }

    bool buttonPressed = digitalRead(BUTTON_PIN) == LOW;
    long readTime = millis();

    if (waitForButtonUp) {
      if (!buttonPressed) {
        waitForButtonUp = false;
      }
    } else {
      if (!buttonPressed && singlePressTime != -1) {
        if (readTime - singlePressTime > doublePressInterval) {
          // double-press timeout
          handleHandler(singlePressHandler);
          singlePressTime = -1;
        }
      }
      if (!buttonPressed && buttonDownTime != -1) {
        if (singlePressTime != -1) {
          // button-up from second press
          handleHandler(doublePressHandler);
          singlePressTime = -1;
        } else {
          singlePressTime = readTime;
        }
      } else if (buttonPressed && buttonDownTime == -1) {
        buttonDownTime = readTime;
      } else if (buttonPressed && readTime - buttonDownTime > longPressInterval) {
        if (singlePressTime != -1) {
          handleHandler(doubleLongPressHandler);
          singlePressTime = -1;
        } else {
          handleHandler(longPressHandler);
        }
        waitForButtonUp = true;
      }
    }

    if (buttonPressed) {
      buttonUpTime = -1;
    } else {
      buttonDownTime = -1;
    }
  }

  void onSinglePress(void (*handler)(void)) {
    singlePressHandler = handler;
  }
 
  void onDoublePress(void (*handler)(void)) {
    doublePressHandler = handler;
  }

  void onLongPress(void (*handler)(void)) {
    longPressHandler = handler;
  }

  void onDoubleLongPress(void (*handler)(void)) {
    doubleLongPressHandler = handler;
  }

  void onThumbdial1(void (*handler)(int)) {
    thumbdial1Handler = handler;
  }

  void onThumbdial2(void (*handler)(int)) {
    thumbdial2Handler = handler;
  }
};

#endif
