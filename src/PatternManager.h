#ifndef PATTERNMANAGER_H
#define PATTERNMANAGER_H

#include <vector>

#include "patterns.h"

/* ---- Test Options ---- */
const long kIdlePatternTimeout = -1;
#define kTestAutomaticModeEnergyChange false

typedef std::function<void(DrawingContext ctx, float progress)> OverlayTick;
struct Overlay {
  unsigned long start;
  unsigned long duration;
  OverlayTick tick;
};

// store a list of "classes" so we can call query static methods in a generic way
class PatternIntrospector {
  template<class T>
  static Pattern *makeConstructor() {
    return Pattern::makePattern<T>();
  }
public:
  PatternFlags flags;
  Pattern * (*construct)(void);

  template<class T>
  static PatternIntrospector Introspect() {
    PatternFlags flags = T::patternFlags();
    Pattern * (*construct)(void) = &(makeConstructor<T>);
    return PatternIntrospector(flags, construct);
  }

  PatternIntrospector(PatternFlags flags, Pattern * (*construct)(void)) : flags(flags), construct(construct) { };
};

template <typename BufferType>
class PatternManager {
  std::vector<PatternIntrospector> allPatterns;

  int activePatternIndex = -1;
  Pattern *activePattern = NULL;  // main pattern
  
  int tempPatternIndex = -1;
  Pattern *tempPattern = NULL; // pattern displaying temporarily during motion event or crossfade

  bool automaticMode = false;

  enum { singlePattern, energyCrossfade, twirlOverlay } compositionState = singlePattern;
  int autoSwitchLastPatternIndex = -1;
  int autoSwitchAccumulator = 0;
  
  DrawingContext overlayCtx;
  std::vector<Overlay> overlays;
  unsigned long lastOverlayActive = 0;

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
      // testIdlePattern = new SpikeSpin();
      // testIdlePattern = new ArrowSpin();
      // testIdlePattern = new Compass();
      // testIdlePattern = new Triangles();
    }
    return testIdlePattern;
  }

public:
  BufferType &ctx;

  PatternManager(BufferType &ctx) : ctx(ctx) {
    allPatterns.push_back(PatternIntrospector::Introspect<ArrowSpin>()); // initial pattern by virtue of being index 0
    allPatterns.push_back(PatternIntrospector::Introspect<PixelDust>());
    allPatterns.push_back(PatternIntrospector::Introspect<Droplets>());
    allPatterns.push_back(PatternIntrospector::Introspect<Oscillators>());
    allPatterns.push_back(PatternIntrospector::Introspect<SpectrumAnalyzer>());
    allPatterns.push_back(PatternIntrospector::Introspect<SmoothPalettes>());
    allPatterns.push_back(PatternIntrospector::Introspect<Compass>());
    allPatterns.push_back(PatternIntrospector::Introspect<Bars>());
    allPatterns.push_back(PatternIntrospector::Introspect<TriangleSpin>());
    allPatterns.push_back(PatternIntrospector::Introspect<SpikeSpin>());
  }

  std::vector<PatternIntrospector> patternsMatchingFlags(PatternFlags flags) {
    std::vector<PatternIntrospector> matching;
    for (auto pattern : allPatterns) {
      if ((pattern.flags & flags) == flags) {
        matching.push_back(pattern);
      }
    }
    return matching;
  }

  int indexOfRandomPatternMatchingFlags(PatternFlags flags) {
    std::vector<std::pair<PatternIntrospector, int> > matching;
    for (unsigned i = 0; i < allPatterns.size(); ++i) {
      auto pattern = allPatterns[i];
      if ((pattern.flags & flags) == flags) {
        matching.push_back(std::pair<PatternIntrospector, int>(pattern, i));
      }
    }
    auto choice = matching[random8(matching.size())];
    return choice.second;
  }

  void RunOverlay(unsigned long durationMillis, OverlayTick tick) {
    Overlay overlay;
    overlay.duration = durationMillis;
    overlay.start = millis();
    overlay.tick = tick;
    overlays.push_back(overlay);
  }

  void setup() {
    // always keep motion running
    motionManager.subscribe();
    
    Pattern *testPattern = TestIdlePattern();
    if (testPattern) {
      setActivePattern(testPattern, -1);
    } else {
      startPatternAtIndex(0);
    }
  }

  void nextPattern() {
    disableAutomaticMode();
    activePatternIndex = (activePatternIndex + 1) % allPatterns.size();
    startPatternAtIndex(activePatternIndex);
  }

  void previousPattern() {
    disableAutomaticMode();
    activePatternIndex = mod_wrap(activePatternIndex - 1, allPatterns.size());
    startPatternAtIndex(activePatternIndex);
  }

  void enableAutomaticMode() {
    automaticMode = true;
  }

  void disableAutomaticMode() {
    automaticMode = false;
    compositionState = singlePattern;
    autoSwitchLastPatternIndex = -1;
    stopTempPattern();
  }

  inline Pattern *createPatternAtIndex(int index) {
    return allPatterns[index].construct();
  }

  void startPatternAtIndex(int index) {
    Pattern *nextPattern = createPatternAtIndex(index);
    setActivePattern(nextPattern, index);
  }

  void setActivePattern(Pattern *pattern, int index) {
    if (activePattern) {
      activePattern->stop();
      delete activePattern;
      activePattern = NULL;
    }

    pattern->start();
    activePattern = pattern;
    activePatternIndex = index;
  }

  void stopTempPattern() {
    if (tempPattern) {
      tempPattern->stop();
      delete tempPattern;
      tempPattern = NULL;
    }
    tempPatternIndex = -1;
  }

  void startTempPatternAtIndex(int index) {
    stopTempPattern();
    Pattern *pattern = createPatternAtIndex(index);
    pattern->start();
    tempPattern = pattern;
    tempPatternIndex = index;
  }

  void handleAutomaticPatternMode() {
    assert(automaticMode == true, "automode");
    // TODO: dial in all these values
    const float kLowEnergyThresh = 10;
    const float kHighEnergyThresh = 60;
    const float kAutoCrossfadeFrameRequirement = 300; // hacky method for progressively crossfading to the high energy pattern by counting the frames with energy above or below threshold
    const float kTwirlThreshold = 1.8;
    const float kTwirlFadeup = 0.7;
    
    motionManager.loop();
    float twirlVelocity = motionManager.twirlVelocity(30);

#if !kTestAutomaticModeEnergyChange
    float bouncyEnergy = motionManager.bouncyEnergy();
#else
    float bouncyEnergy = beatsin8(1, 0, 70);
    EVERY_N_MILLIS(500) {
      logf("test bouncyEnergy = %i", (int)bouncyEnergy);
    }
#endif

    if (compositionState == singlePattern) {
      if (activePattern && !(activePattern->flags & patternFlagHighEnergy) && bouncyEnergy > kHighEnergyThresh) {
        logf("long sustained motion, starting dust");
        // pick a high energy pattern at random
        int choice = indexOfRandomPatternMatchingFlags(patternFlagHighEnergy);
        startTempPatternAtIndex(choice);
        autoSwitchAccumulator = 0;
        compositionState = energyCrossfade;
      } else if (autoSwitchLastPatternIndex != -1 && bouncyEnergy < kLowEnergyThresh) {
        logf("long sustained low motion, reverting to last pattern %i", autoSwitchLastPatternIndex);
        startTempPatternAtIndex(autoSwitchLastPatternIndex);
        autoSwitchAccumulator = 0;
        compositionState = energyCrossfade;
      }
    }
    if (compositionState == energyCrossfade) {
      if (autoSwitchLastPatternIndex != -1) {
        autoSwitchAccumulator += (bouncyEnergy < kHighEnergyThresh ? 1 : -1);
      } else {
        autoSwitchAccumulator += (bouncyEnergy > kHighEnergyThresh ? 1 : -1);
      }

      if (autoSwitchAccumulator < -90) {
        logf("autoswitch crossfade cancel");
        stopTempPattern();
        autoSwitchAccumulator = 0;
        compositionState = singlePattern;
      } else if (autoSwitchAccumulator > kAutoCrossfadeFrameRequirement) {
        logf("autoswitch crossfade finished");
        if (activePattern && tempPattern) {
          autoSwitchLastPatternIndex = (autoSwitchLastPatternIndex == -1 ? activePatternIndex : -1);
          activePattern->stop();
          delete activePattern;
          activePattern = tempPattern;
          activePatternIndex = tempPatternIndex;
          tempPattern = NULL;
          tempPatternIndex = -1;
          autoSwitchAccumulator = 0;
          activePattern->setBrightness(0xFF);
        }
        compositionState = singlePattern;
      } else {
        if (activePattern) {
          activePattern->setBrightness(0xFF - 0xFF * max(0,autoSwitchAccumulator) / kAutoCrossfadeFrameRequirement);
        }
        if (tempPattern) {
          tempPattern->setBrightness(0xFF * max(0,autoSwitchAccumulator) / kAutoCrossfadeFrameRequirement);
        }
      }
    }

    if (compositionState == singlePattern && activePattern && !(activePattern->flags & patternFlagTwirl)) {
      if (fabsf(twirlVelocity) > kTwirlThreshold) {
        // choose a twirl pattern at random
        // TODO: can this sort of slowly alternate between choices instead?
        int choice = indexOfRandomPatternMatchingFlags(patternFlagTwirl);
        startTempPatternAtIndex(choice);
        if (tempPattern) {
          tempPattern->setBrightness(0);
        }
        compositionState = twirlOverlay;
      }
    }
    if (compositionState == twirlOverlay) {
      if (fabsf(twirlVelocity) < 0.8 * kTwirlThreshold && tempPattern->brightness == 0) {
        // stop twirl
        stopTempPattern();
        if (activePattern) {
          activePattern->setBrightness(0xFF, true);
        }
        compositionState = singlePattern;
      } else if (activePattern && tempPattern) {
        float twirlFade = (fabsf(twirlVelocity) - kTwirlThreshold) / kTwirlFadeup;
        tempPattern->setBrightness(min(0xFF, max(0, 0xFF * (twirlFade + 0.1))), true, 3);
        activePattern->setBrightness(min(0xFF, max(0, 0xFF - 0xFF * (twirlFade + 0.8))), true, 4);
      }
    }
  }

  void loop() {
    ctx.leds.fill_solid(CRGB::Black);
    
    // TODO: automatic mode being on at all is current dropping us about 6fps from 78 down to 72, where's all that time going?
    if (automaticMode) {
      handleAutomaticPatternMode();
    }

    if (activePattern) {
      activePattern->loop();
      activePattern->composeIntoContext(ctx);
    }
    if (tempPattern) {
      tempPattern->loop();
      tempPattern->composeIntoContext(ctx);
    }

    for (auto it = overlays.begin(); it != overlays.end(); ) {
      lastOverlayActive = millis();
      Overlay overlay = *it;
      if (overlay.start + overlay.duration < millis()) {
        overlays.erase(it);
      } else {
        float progress = min(1.0, max(0.0, (millis() - overlay.start) / (float)overlay.duration));
        overlay.tick(overlayCtx, progress);
        ++it;
      }
    }

    if (millis() - lastOverlayActive < 500) {
      overlayCtx.blendIntoContext(ctx, BlendMode::blendBrighten);
      overlayCtx.leds.fadeToBlackBy(30);
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
      int choice = (int)random8(allPatterns.size());
      startPatternAtIndex(choice);
    }
  }
};

#endif
