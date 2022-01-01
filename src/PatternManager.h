#ifndef PATTERNMANAGER_H
#define PATTERNMANAGER_H

#include <vector>

#include "patterns.h"

/* ---- Test Options ---- */
const bool kTestPatternTransitions = false;
const long kIdlePatternTimeout = -1;//1000 * (kTestPatternTransitions ? 20 : 60 * 2);

typedef std::function<void(DrawingContext ctx, float progress)> OverlayTick;
struct Overlay {
  unsigned long start;
  unsigned long duration;
  OverlayTick tick;
}; 

template <typename BufferType>
class PatternManager {
  int patternIndex = -1;
  Pattern *activePattern = NULL;

  std::vector<Pattern * (*)(void)> patternConstructors;
  
  DrawingContext overlayCtx;
  std::vector<Overlay> overlays;

  bool automaticPatternSwitching = false;

  // track the last known low-motion event for auto-switching
  // TODO: make better, more general, use cross fades
  unsigned long lastLowEnergy = 0;

  int initialPatternIndex = 0;
  int highMotionPatternIndex = 0;

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
    }
    return testIdlePattern;
  }

public:
  BufferType &ctx;

  PatternManager(BufferType &ctx) : ctx(ctx) {
    patternConstructors.push_back(&(construct<Compass>));
    patternConstructors.push_back(&(construct<Bars>));
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
    startPatternAtIndex(initialPatternIndex);
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

  void toggleAutoPattern() {
    automaticPatternSwitching = !automaticPatternSwitching;
    logf("Pattern Automatic -> %s", (automaticPatternSwitching ? "ON" : "OFF"));
    
    RunOverlay(500, [this](DrawingContext ctx, float progress) {
      float x1 = automaticPatternSwitching ? progress * PANEL_WIDTH : PANEL_WIDTH - progress * PANEL_WIDTH;
      float x2 = automaticPatternSwitching ? 2*PANEL_WIDTH - progress * PANEL_WIDTH : PANEL_WIDTH + progress * PANEL_WIDTH;
      ctx.line(x1, 0, x1, PANEL_HEIGHT-1, CRGB::White);
      ctx.line(x2, 0, x2, PANEL_HEIGHT-1, CRGB::White);
    });
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

    if (automaticPatternSwitching) {
      // TODO: dial in all these values
      const float kEnergyThresh = 60;
      
      motionManager.loop();
      if (motionManager.bouncyEnergy() < kEnergyThresh) {
        lastLowEnergy = millis();
      }
      // logf("motionManager.bouncyEnergy() = %f", motionManager.bouncyEnergy());
      if (lastLowEnergy != 0 && millis() - lastLowEnergy > 5000) {
        // 5 seconds of sustained motion
        if (patternIndex != highMotionPatternIndex) {
          logf("long sustained motion, switching to dust");
          startPatternAtIndex(highMotionPatternIndex);
          lastLowEnergy = 0;
        }
      }
    }

    if (activePattern) {
      activePattern->loop();
      activePattern->ctx.blendIntoContext(ctx, BlendMode::blendBrighten, 0xFF);
    }

    for (auto it = overlays.begin(); it != overlays.end(); ) {
      Overlay overlay = *it;
      if (overlay.start + overlay.duration < millis()) {
        overlays.erase(it);
      } else {
        float progress = min(1.0, max(0.0, (millis() - overlay.start) / (float)overlay.duration));
        overlay.tick(overlayCtx, progress);
        ++it;
      }
    }
    overlayCtx.blendIntoContext(ctx, BlendMode::blendBrighten);
    overlayCtx.leds.fadeToBlackBy(30);

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
