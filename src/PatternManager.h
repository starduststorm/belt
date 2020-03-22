#ifndef PATTERNMANAGER_H
#define PATTERNMANAGER_H

#include <vector>

#include "patterns.h"

/* ---- Test Options ---- */
const bool kTestPatternTransitions = false;
const long kIdlePatternTimeout = -1;//1000 * (kTestPatternTransitions ? 20 : 60 * 2);

// Pattern *testIdlePattern = NULL;
// Pattern *testIdlePattern = new Oscillators();
// Pattern *testIdlePattern = new Sound();
// Pattern *testIdlePattern = new Bars());
Pattern *testIdlePattern = new Droplets();

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
    patternConstructors.push_back(&(construct<Droplets>));
    
    // patternConstructors.push_back(&(PatternManager::construct<Bits>));
    // patternConstructors.push_back(&(PatternManager::construct<Motion>));
  }

  void nextPattern() {
    if (activePattern) {
      activePattern->stop();
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
    if (startPattern(nextPattern)) {
      patternIndex = index;
      return true;
    } else {
      delete nextPattern; // patternConstructors returns retained
      return false;
    }
  }

  bool startPattern(Pattern *pattern) {
    if (pattern->wantsToRun()) {
      pattern->start(*drawingContext);
      activePattern = pattern;
      return true;
    } else {
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
        activePattern->stop();
        delete activePattern;
        activePattern = NULL;
      }
    }

    // start a new random pattern if there is none
    if (activePattern == NULL) {
      if (testIdlePattern) {
        startPattern(testIdlePattern);
      } else {
        int choice = (int)random8(patternConstructors.size());
        startPatternAtIndex(choice);
      }
    }
  }
};

#endif