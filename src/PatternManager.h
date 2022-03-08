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

template <typename BufferType>
class PatternManager {
  std::vector<Pattern * (*)(void)> patternConstructors;

  int activePatternIndex = -1;
  Pattern *activePattern = NULL;  // main pattern
  
  int tempPatternIndex = -1;
  Pattern *tempPattern = NULL; // pattern displaying temporarily during motion event or crossfade

  bool automaticPatternSwitching = false;
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

    // "constants"
  int initialPatternIndex = 0;
  int highMotionPatternIndex = 0;

public:
  BufferType &ctx;

  PatternManager(BufferType &ctx) : ctx(ctx) {
    patternConstructors.push_back(&(construct<Compass>));
    patternConstructors.push_back(&(construct<Bars>));
    patternConstructors.push_back(&(construct<Triangles>));
    patternConstructors.push_back(&(construct<SpikeSpin>));
    patternConstructors.push_back(&(construct<ArrowSpin>)); initialPatternIndex = patternConstructors.size()-1;
    patternConstructors.push_back(&(construct<PixelDust>)); highMotionPatternIndex = patternConstructors.size()-1;
    patternConstructors.push_back(&(construct<Droplets>));
    patternConstructors.push_back(&(construct<Oscillators>));
    patternConstructors.push_back(&(construct<SpectrumAnalyzer>));
    patternConstructors.push_back(&(construct<SmoothPalettes>));
    
    // patternConstructors.push_back(&(PatternManager::construct<Bits>));
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
      startPatternAtIndex(initialPatternIndex);
    }
  }

  void nextPattern() {
    if (automaticPatternSwitching) toggleAutoPattern();

    activePatternIndex = (activePatternIndex + 1) % patternConstructors.size();
    if (!startPatternAtIndex(activePatternIndex)) {
      nextPattern();
    }
  }

  void previousPattern() {
    if (automaticPatternSwitching) toggleAutoPattern();

    activePatternIndex = mod_wrap(activePatternIndex - 1, patternConstructors.size());
    if (!startPatternAtIndex(activePatternIndex)) {
      nextPattern();
    }
  }

  // TODO: change the button action to always just "enable" and show a more intresting modal instead of this boring overlay. main.cpp can run the Modal when the button is pressed only.
  void toggleAutoPattern(bool runOverlay=false) {
    automaticPatternSwitching = !automaticPatternSwitching;
    if (!automaticPatternSwitching) {
      autoSwitchLastPatternIndex = -1;
    }
    logf("Pattern Automatic -> %s", (automaticPatternSwitching ? "ON" : "OFF"));
    if (runOverlay) {
      RunOverlay(500, [this](DrawingContext ctx, float progress) {
        int maxX = PANEL_WIDTH - 1;
        float x1 = automaticPatternSwitching ? progress * maxX : maxX - progress * maxX;
        float x2 = automaticPatternSwitching ? 2*maxX - progress * maxX : maxX + progress * maxX;
        ctx.line(x1, 0, x1, PANEL_HEIGHT-1, CRGB::White);
        ctx.line(x2, 0, x2, PANEL_HEIGHT-1, CRGB::White);
      });
    }
  }

  inline Pattern *createPatternAtIndex(int index) {
    auto ctor = patternConstructors[index];
    return ctor();
  }

  bool startPatternAtIndex(int index) {
    Pattern *nextPattern = createPatternAtIndex(index);
    if (setActivePattern(nextPattern, index)) {
      return true;
    }
    delete nextPattern; // patternConstructors returns retained
    return false;
  }

  bool setActivePattern(Pattern *pattern, int index) {
    if (tempPattern) {
      tempPattern->stop();
      delete tempPattern;
      tempPattern = NULL;
    }

    if (activePattern) {
      activePattern->stop();
      delete activePattern;
      activePattern = NULL;
    }

    if (pattern->wantsToRun()) {
      pattern->start();
      activePattern = pattern;
      activePatternIndex = index;
      return true;
    } else {
      return false;
    }
  }

  void startTempPatternAtIndex(int index) {
    if (tempPattern) {
      tempPattern->stop();
      delete tempPattern;
      tempPattern = NULL;
    }
    
    Pattern *pattern = createPatternAtIndex(index);
    if (pattern && pattern->wantsToRun()) {
      pattern->start();
      tempPattern = pattern;
      tempPatternIndex = index;
    } else if (pattern) {
      delete pattern;
    }
  }

  void loop() {
    ctx.leds.fill_solid(CRGB::Black);

    const float kAutoCrossfadeFrameRequirement = 300; // hacky method for progressively crossfading to the high energy pattern by counting the frames with energy above or below threshold
    
    // TODO: automatic mode being on at all is current dropping us about 6fps from 78 down to 72, where's all that time going?
    if (automaticPatternSwitching) {
      // TODO: dial in all these values
      const float kLowEnergyThresh = 10;
      const float kHighEnergyThresh = 60;
      
      motionManager.loop();
#if !kTestAutomaticModeEnergyChange
      float bouncyEnergy = motionManager.bouncyEnergy();
#else
      float bouncyEnergy = beatsin8(1, 0, 70);
      EVERY_N_MILLIS(500) {
        logf("test bouncyEnergy = %i", (int)bouncyEnergy);
      }
#endif

      // EVERY_N_MILLIS(1000) {
      //   logf("bouncyEnergy = %f, autoSwitchAccumulator = %i", bouncyEnergy, autoSwitchAccumulator);
      // }
      if (activePatternIndex != highMotionPatternIndex) {
        if (bouncyEnergy > kHighEnergyThresh) {
          if (!tempPattern) {
            logf("long sustained motion, starting dust");
            startTempPatternAtIndex(highMotionPatternIndex);
            autoSwitchAccumulator = 0;
          }
          autoSwitchAccumulator++;
        } else if (tempPattern) {
          autoSwitchAccumulator--;
        }
      } else if (autoSwitchLastPatternIndex != -1) {
        if (bouncyEnergy < kLowEnergyThresh) {
          if (!tempPattern) {
            logf("long sustained low motion, reverting to last pattern %i", autoSwitchLastPatternIndex);
            startTempPatternAtIndex(autoSwitchLastPatternIndex);
            autoSwitchAccumulator = 0;
          }
          autoSwitchAccumulator++;
        } else if (tempPattern) {
          autoSwitchAccumulator--;
        }
      }

      // cancel auto switch
      if (tempPattern && autoSwitchAccumulator < -90) {
        logf("autoswitch crossfade cancel");
        tempPattern->stop();
        delete tempPattern;
        tempPattern = NULL;
        autoSwitchAccumulator = 0;
      }

      if (tempPattern && autoSwitchAccumulator > kAutoCrossfadeFrameRequirement) {
        // we've finished the crossfade
        logf("autoswitch crossfade finished");
        if (activePattern) {
          autoSwitchLastPatternIndex = activePatternIndex;
          activePattern->stop();
          delete activePattern;
          activePattern = tempPattern;
          activePatternIndex = tempPatternIndex;
          tempPattern = NULL;
          tempPatternIndex = -1;
          autoSwitchAccumulator = 0;
        }
      }
    }

    if (activePattern) {
      activePattern->loop();
      activePattern->ctx.blendIntoContext(ctx, BlendMode::blendBrighten, 0xFF - 0xFF * max(0,autoSwitchAccumulator) / kAutoCrossfadeFrameRequirement);
    }
    if (tempPattern) {
      tempPattern->loop();
      tempPattern->ctx.blendIntoContext(ctx, BlendMode::blendBrighten, 0xFF * max(0,autoSwitchAccumulator) / kAutoCrossfadeFrameRequirement);
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
      int choice = (int)random8(patternConstructors.size());
      startPatternAtIndex(choice);
    }
  }
};

#endif
