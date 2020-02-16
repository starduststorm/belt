#define FASTLED_ALLOW_INTERRUPTS 0

// FIXME: workaround for build issue in FastLED when FASTLED_ALLOW_INTERRUPTS is 0
#ifndef MS_COUNTER 
#define MS_COUNTER systick_millis_count
#endif
// end FIXME

#define DEBUG 1
#define USE_AUDIO 0

#include <FastLED.h>

#define SERIAL_LOGGING 1
#define PANEL_WIDTH 32
#define PANEL_HEIGHT 8
#define PANEL_COUNT 2

// FIXME: remove:
#define STRIP_LENGTH PANEL_HEIGHT
#define STRIP_COUNT PANEL_WIDTH
//

#define PANEL_LEDS (PANEL_WIDTH * PANEL_HEIGHT)
#define NUM_LEDS (PANEL_LEDS * PANEL_COUNT)

// Pin 23 is the microphone pin, so we may be seeding random with mic noise
#define UNCONNECTED_PIN_1 15 // FIXME: now used for LEDS
#define UNCONNECTED_PIN_2 23

#include "util.h"
#include "patterns.h"
#include "AudioManager.h"

/* ---- Options ---- */
// FIXME: should have options here for mounting, e.g. side-down vs. corner down, which strand is top/bottom, etc.
// though changing these flags in the field is not very practical if it requires a recompile.
/* ---- ------------*/

CRGBArray<NUM_LEDS> leds;

Droplets dropletsPattern;
Bits bitsPattern;
SmoothPalettes smoothPalettes;
#if USE_AUDIO
Sound soundPattern;
#endif
RaverPlaid raverPlaid;
Motion motion;
PixelDust pixelDust;

Pattern *idlePatterns[] = {
                            &bitsPattern, &dropletsPattern, &smoothPalettes, &raverPlaid, &motion, &pixelDust,
#if USE_AUDIO
                            &soundPattern
#endif
                          };
const unsigned int kIdlePatternsCount = ARRAY_SIZE(idlePatterns);

Pattern *activePattern = NULL;
Pattern *lastPattern = NULL;

/* ---- Test Options ---- */
const bool kTestPatternTransitions = true;
const int kIdlePatternTimeout = 1000 * (kTestPatternTransitions ? 10 : 60 * 2);

Pattern *testIdlePattern = &pixelDust;

/* ---------------------- */

unsigned long lastTrigger = 0;
FrameCounter fc;

/* 
 * These need to be specific pins on Teensy 4.0
 * https://github.com/FastLED/FastLED/wiki/Parallel-Output
 * First: 1,0,24,25,19,18,14,15,17,16,22,23,20,21,26,27
 * Second: 10,12,11,13,6,9,32,8,7
 * Third: 37, 36, 35, 34, 39, 38, 28, 31, 30
 */
#define DATA_PIN_1 14
#define DATA_PIN_2 15

//#define MIC_PIN 23

int lsb_noise(int pin, int numbits) {
  // TODO: Use Entropy.h? Probs not needed just to randomize pattern.
  int noise = 0;
  for (int i = 0; i < numbits; ++i) {
    int val = analogRead(pin);
    noise = (noise << 1) | (val & 1);
  }
  return noise;
}

void setup() {
  Serial.begin(57600);
  while (!Serial);
  Serial.println("begin");
  
  randomSeed(lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint32_t)));
  random16_add_entropy(lsb_noise(UNCONNECTED_PIN_2, 8 * sizeof(uint16_t)));
  
  FastLED.addLeds<PANEL_COUNT, WS2812B, DATA_PIN_1, GRB>(leds, PANEL_LEDS).setCorrection(TypicalSMD5050);
  LEDS.setBrightness(255);

  fc.tick();
}

void loop() {
  for (unsigned i = 0; i < kIdlePatternsCount; ++i) {
    Pattern *pattern = idlePatterns[i];
    if (pattern->isRunning() || pattern->isStopping()) {
      pattern->loop(leds);
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
      nextPattern->start();
      activePattern = nextPattern;
    }
  }

  FastLED.show();
  
  fc.tick();
  fc.clampToFramerate(60);

}
