#ifndef CONTROLS_H
#define CONTROLS_H

#define THUMBDIAL1_PIN A8
#define THUMBDIAL2_PIN A2
#define BUTTON_PIN 0

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

  void handleHandler(void (*handler)(void)) {
    if (handler) {
      (*handler)();
    }
  }  
  
public:
  long longPressInterval = 1000;
  long doublePressInterval = 400;

  Controls() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
  }

  void update() {
    // TODO: move dials to handlers, generalize all this for different hardware layouts
    int brightness = map(analogRead(THUMBDIAL1_PIN), 1023, 0, 0, 0xFF);
    LEDS.setBrightness(brightness);

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
};

#endif
