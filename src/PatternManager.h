#ifndef PATTERNMANAGER_H
#define PATTERNMANAGER_H

#include <EEPROM.h>
#include "patterns.h"

Droplets dropletsPattern;
Bits bitsPattern;
SmoothPalettes smoothPalettes;
Motion motion;
PixelDust pixelDust;
Bars barsPattern;
Oscillators oscillatorsPattern;
Sound soundPattern;

/* ---- Test Options ---- */
const bool kTestPatternTransitions = false;
const long kIdlePatternTimeout = -1;//1000 * (kTestPatternTransitions ? 20 : 60 * 2);

Pattern *testIdlePattern = NULL;
// Pattern *testIdlePattern = &oscillatorsPattern;
// Pattern *testIdlePattern = &soundPattern;
// Pattern *testIdlePattern = &barsPattern;

Pattern *idlePatterns[] = {
                            &smoothPalettes, &pixelDust, &barsPattern, &oscillatorsPattern, &soundPattern
                          };
const unsigned int kIdlePatternsCount = ARRAY_SIZE(idlePatterns);

class PatternManager {
  int patternIndex = -1;
  Pattern *activePattern = NULL;

  std::vector<Pattern * (*)(void)> patternConstructors;

  template<class T>
  static Pattern *construct() {
    return new T();
  }

public:
  DrawingContext *drawingContext;

  PatternManager() {
    patternConstructors.push_back(&(construct<SmoothPalettes>));
    patternConstructors.push_back(&(construct<PixelDust>));
    patternConstructors.push_back(&(construct<Bars>));
    patternConstructors.push_back(&(construct<Oscillators>));
    patternConstructors.push_back(&(construct<Sound>));

    // patternConstructors.push_back(&(PatternManager::construct<Droplets>));
    // patternConstructors.push_back(&(PatternManager::construct<Bits>));
    // patternConstructors.push_back(&(PatternManager::construct<Motion>));
  }

  void nextPattern() {
    if (activePattern) {
      activePattern->lazyStop();
      delete activePattern;
      activePattern = NULL;
    }

    patternIndex = (patternIndex + 1) % patternConstructors.size();
    if (!startPatternAtIndex(patternIndex)) {
      nextPattern();
    }
  }

  bool startPatternAtIndex(int index) {
    auto ctor = patternConstructors[index];
    Pattern *nextPattern = ctor();
    if (nextPattern->wantsToRun()) {
      nextPattern->start(*drawingContext);
      activePattern = nextPattern;
      patternIndex = index;
      return true;
    } else {
      delete nextPattern;
      return false;
    }
  }

  void loop() {
    if (activePattern) {
      activePattern->loop(*drawingContext);
    }

    // time out idle patterns
    if (activePattern != NULL && kIdlePatternTimeout != -1 && activePattern->isRunning() && activePattern->runTime() > kIdlePatternTimeout) {
      if (activePattern != testIdlePattern && activePattern->wantsToIdleStop()) {
        activePattern->lazyStop();
        delete activePattern;
        activePattern = NULL;
      }
    }

    // start a new random pattern if there is none
    if (activePattern == NULL) {
      int choice = (int)random8(patternConstructors.size());
      startPatternAtIndex(choice);
    }
  }
};

#endif
