#ifndef PATTERNMANAGER_H
#define PATTERNMANAGER_H

#include <vector>

#include "patterns.h"

/* ---- Test Options ---- */
const bool kTestPatternTransitions = false;
const long kIdlePatternTimeout = -1;//1000 * (kTestPatternTransitions ? 20 : 60 * 2);

template <typename BufferType>
class PatternManager {
  int patternIndex = -1;
  Pattern *activePattern = NULL;

  std::vector<Pattern * (*)(void)> patternConstructors;

  template<class T>
  static Pattern *construct() {
    return new T();
  }

  // Make testIdlePattern in this constructor instead of at global so the Pattern doesn't get made at launch
  Pattern *TestIdlePattern() {
    static Pattern *testIdlePattern = NULL;
    if (testIdlePattern == NULL) {
      // testIdlePattern = NULL;
      // testIdlePattern = new Oscillators();
      // testIdlePattern = new SpectrumAnalyzer();
      // testIdlePattern = new Bars());
      // testIdlePattern = new Droplets();
      // testIdlePattern = new PixelDust();
      // testIdlePattern = new MotionBlobs();
      // testIdlePattern = new Compass();
    }
    return testIdlePattern;
  }

public:
  BufferType &ctx;

  PatternManager(BufferType &ctx) : ctx(ctx) {
    patternConstructors.push_back(&(construct<Compass>));
    patternConstructors.push_back(&(construct<MotionBlobs>));
    patternConstructors.push_back(&(construct<Bars>));
    patternConstructors.push_back(&(construct<PixelDust>));
    patternConstructors.push_back(&(construct<SpectrumAnalyzer>));
    patternConstructors.push_back(&(construct<Oscillators>));
    patternConstructors.push_back(&(construct<Droplets>));
    patternConstructors.push_back(&(construct<SmoothPalettes>));
    
    // patternConstructors.push_back(&(PatternManager::construct<Bits>));
  }

  void nextPattern() {
    patternIndex = (patternIndex + 1) % patternConstructors.size();
    if (!startPatternAtIndex(patternIndex)) {
      nextPattern();
    }
  }

  void previousPattern() {
    patternIndex = mod_wrap(patternIndex - 1, patternConstructors.size());
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
    if (activePattern) {
      activePattern->stop();
      delete activePattern;
      activePattern = NULL;
    }

    if (pattern->wantsToRun()) {
      pattern->start();
      activePattern = pattern;
      return true;
    } else {
      return false;
    }
  }

  void loop() {
    ctx.leds.fill_solid(CRGB::Black);

    if (activePattern) {
      activePattern->loop();

      activePattern->ctx.blendIntoContext(ctx, BlendMode::blendBrighten, 0xFF);
    }

    // time out idle patterns
    if (activePattern != NULL && kIdlePatternTimeout != -1 && activePattern->isRunning() && activePattern->runTime() > kIdlePatternTimeout) {
      if (activePattern != TestIdlePattern() && activePattern->wantsToIdleStop()) {
        activePattern->stop();
        delete activePattern;
        activePattern = NULL;
      }
    }

    // start a new random pattern if there is none
    if (activePattern == NULL) {
      Pattern *testPattern = TestIdlePattern();
      if (testPattern) {
        startPattern(testPattern);
      } else {
        int choice = (int)random8(patternConstructors.size());
        startPatternAtIndex(choice);
      }
    }
  }
};

#endif
