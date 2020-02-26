#define FASTLED_ALLOW_INTERRUPTS 0

// FIXME: workaround for build issue in FastLED when FASTLED_ALLOW_INTERRUPTS is 0
#ifndef MS_COUNTER 
#define MS_COUNTER systick_millis_count
#endif
// end FIXME

#define DEBUG 1
#define WAIT_FOR_SERIAL 1
#define USE_AUDIO 1

#include <FastLED.h>

#define SERIAL_LOGGING 1
#define PANEL_WIDTH 32
#define PANEL_HEIGHT 8
#define PANEL_COUNT 2
#define TOTAL_WIDTH PANEL_COUNT * PANEL_WIDTH
#define TOTAL_HEIGHT PANEL_HEIGHT

#define PANEL_LEDS (PANEL_WIDTH * PANEL_HEIGHT)
#define NUM_LEDS (PANEL_LEDS * PANEL_COUNT)

#include "util.h"
#include "patterns.h"
#include "AudioManager.h"
#include "controls.h"

CRGBArray<NUM_LEDS> leds;
DrawingContext *drawingContext;

Droplets dropletsPattern;
Bits bitsPattern;
SmoothPalettes smoothPalettes;
Motion motion;
PixelDust pixelDust;
Bars barsPattern;
Oscillators oscillatorsPattern;
#if USE_AUDIO
Sound soundPattern;
#endif

Pattern *idlePatterns[] = {
                            &bitsPattern, &dropletsPattern, &smoothPalettes, &motion, &pixelDust, &barsPattern, &oscillatorsPattern,
#if USE_AUDIO
                            &soundPattern
#endif
                          };
const unsigned int kIdlePatternsCount = ARRAY_SIZE(idlePatterns);

Pattern *activePattern = NULL;
Pattern *lastPattern = NULL;

/* ---- Test Options ---- */
const bool kTestPatternTransitions = true;
const int kIdlePatternTimeout = 1000 * (kTestPatternTransitions ? 20 : 60 * 2);

Pattern *testIdlePattern = &barsPattern;

/* ---------------------- */

unsigned long lastTrigger = 0;
FrameCounter fc;
Controls controls;

#define UNCONNECTED_PIN_1 A9
#define UNCONNECTED_PIN_2 A3

/* 
 * These need to be specific pins on Teensy 4.0
 * https://github.com/FastLED/FastLED/wiki/Parallel-Output
 * First: 1,0,24,25,19,18,14,15,17,16,22,23,20,21,26,27
 * Second: 10,12,11,13,6,9,32,8,7
 * Third: 37, 36, 35, 34, 39, 38, 28, 31, 30
 */
#define DATA_PIN_1 14
#define DATA_PIN_2 15

void applyBrightnessSettings();

int lsb_noise(int pin, int numbits) {
  // TODO: Use Entropy.h? Probs not needed just to randomize pattern.
  int noise = 0;
  for (int i = 0; i < numbits; ++i) {
    int val = analogRead(pin);
    noise = (noise << 1) | (val & 1);
  }
  return noise;
}

void buttonSinglePress() {
  logf("single press!");
  // TODO: make deterministic?
  if (activePattern != NULL && activePattern->isRunning()) {
    if (activePattern != testIdlePattern && activePattern->wantsToIdleStop()) {
      activePattern->lazyStop();
      lastPattern = activePattern;
      activePattern = NULL;
    }
  }
}

void buttonDoublePress() {
  logf("double press!");
}

void buttonLongPress() {
  logf("long press!");
}

void buttonDoubleLongPress() {
  logf("double long press");
}

void setup() {
  Serial.begin(57600);
#if WAIT_FOR_SERIAL
  long setupStart = millis();
  while (!Serial);
  long serialReady = millis();
  logf("begin - waited %0.2fs for Serial", (serialReady - setupStart) / 1000.);
#elif DEBUG
  delay(2000);
#endif
  
  randomSeed(lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint32_t)));
  random16_add_entropy(lsb_noise(UNCONNECTED_PIN_2, 8 * sizeof(uint16_t)));

  // FIXME: TypicalSMD5050 color correction produces reds at very low brightness levels, such that fading a white pixel down to black produces red at the end.
  // This is very undesirable, so I'd rather use uncorrected colors .setCorrection((UncorrectedColor).
  //.setCorrection(TypicalSMD5050);
  FastLED.addLeds<PANEL_COUNT, WS2812B, DATA_PIN_1, GRB>(leds, PANEL_LEDS);

  drawingContext = new DrawingContext(leds, TOTAL_WIDTH, TOTAL_HEIGHT);

  fc.tick();
  controls.onSinglePress(&buttonSinglePress);
  controls.onDoublePress(&buttonDoublePress);
  controls.onLongPress(&buttonLongPress);
  controls.onDoubleLongPress(&buttonDoubleLongPress);
  controls.update();
}

void loop() {
  for (unsigned i = 0; i < kIdlePatternsCount; ++i) {
    Pattern *pattern = idlePatterns[i];
    if (pattern->isRunning() || pattern->isStopping()) {
      pattern->loop(*drawingContext);
    }
  }

  // clear out patterns that have stopped themselves
  if (activePattern != NULL && !activePattern->isRunning()) {
    logf("Clearing inactive pattern %s", activePattern->description());
    activePattern = NULL;
  }

  // time out idle patterns
  if (activePattern != NULL && activePattern->isRunning() && activePattern->runTime() > kIdlePatternTimeout) {
    if (activePattern != testIdlePattern && activePattern->wantsToIdleStop()) {
      activePattern->lazyStop();
      lastPattern = activePattern;
      activePattern = NULL;
    }
  }

  // start a new idle pattern
  if (activePattern == NULL) {
    Pattern *nextPattern;
    if (testIdlePattern != NULL) {
      nextPattern = testIdlePattern;
    } else {
      int choice = (int)random8(kIdlePatternsCount);
      nextPattern = idlePatterns[choice];
    }
    if ((nextPattern != lastPattern || nextPattern == testIdlePattern || kIdlePatternsCount == 1) && !nextPattern->isRunning() && nextPattern->wantsToRun()) {
      nextPattern->start(*drawingContext);
      activePattern = nextPattern;
    }
  }
  
  applyBrightnessSettings();
  
  FastLED.show();
  
  fc.tick();
  fc.clampToFramerate(90);
  controls.update();
}

void applyBrightnessSettings() {
    static long firstLoopMillis = 0;
  const long fadeInTime = 2000;
  if (firstLoopMillis == 0) {
    firstLoopMillis = millis();
  }
  long earlyRunTime = firstLoopMillis + fadeInTime - millis();
  if (earlyRunTime > 0) {
    leds.fadeLightBy(0xFF * earlyRunTime / (float)fadeInTime);
  }
}
