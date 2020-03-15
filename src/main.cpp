#define FASTLED_ALLOW_INTERRUPTS 0

// FIXME: workaround for build issue in FastLED when FASTLED_ALLOW_INTERRUPTS is 0
#ifndef MS_COUNTER 
#define MS_COUNTER systick_millis_count
#endif
// end FIXME

#define DEBUG 1
#define WAIT_FOR_SERIAL 0

#include <FastLED.h>

#define PANEL_WIDTH 32
#define PANEL_HEIGHT 8
#define PANEL_COUNT 2
#define TOTAL_WIDTH PANEL_COUNT * PANEL_WIDTH
#define TOTAL_HEIGHT PANEL_HEIGHT

#define PANEL_LEDS (PANEL_WIDTH * PANEL_HEIGHT)
#define NUM_LEDS (PANEL_LEDS * PANEL_COUNT)

#include "util.h"
#include "PatternManager.h"
#include "AudioManager.h"
#include "controls.h"

CRGBArray<NUM_LEDS> leds;
DrawingContext *drawingContext;

/* ---------------------- */

FrameCounter fc;
HardwareControls controls;
PatternManager patternManager;

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

#define THUMBDIAL1_PIN A8
#define THUMBDIAL2_PIN A2
#define BUTTON_PIN 0

void buttonSinglePress() {
  patternManager.nextPattern();
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

static uint8_t globalBrightness = 0xFF;
void thumbdial1Change(int val) {
  val = map(val, 1023, 0, 0, 0xFF);
  globalBrightness = val;
}

static bool serialTimeout = false;
static unsigned long setupDoneTime;

void setup() {
  Serial.begin(57600);
#if WAIT_FOR_SERIAL
  long setupStart = millis();
  while (!Serial) {
    if (millis() - setupStart > 5000) {
      serialTimeout = true;
      break;
    }
  }
  logf("begin - waited %0.2fs for Serial", (millis()- setupStart) / 1000.);
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
  patternManager.drawingContext = drawingContext;

  fc.tick();
  SPSTButton *button = controls.addButton(BUTTON_PIN);
  button->onSinglePress(&buttonSinglePress);
  button->onDoublePress(&buttonDoublePress);
  button->onLongPress(&buttonLongPress);
  button->onDoubleLongPress(&buttonDoubleLongPress);

  AnalogDial *brightnessDial = controls.addAnalogDial(THUMBDIAL1_PIN);
  brightnessDial->onChange(&thumbdial1Change);
  
  controls.update();

  setupDoneTime = millis();
}

void serialTimeoutIndicator() {
  leds.fill_solid(CRGB::Black);
  if ((millis() - setupDoneTime) % 250 < 100) {
    for (int p = 0; p < PANEL_COUNT; ++p) {
      drawingContext->line(p*PANEL_WIDTH, 0, p*PANEL_WIDTH, PANEL_HEIGHT-1, CRGB::Red);
    }
  }
  FastLED.show();
  delay(20);
}

void loop() {
  if (serialTimeout && millis() - setupDoneTime < 1000) {
    serialTimeoutIndicator();
    return;
  }

  patternManager.loop();
   
  applyBrightnessSettings();
  
  // Rotate Panel 0 since it's mounted upside down
  CRGBArray<PANEL_LEDS> panelCopy;
  panelCopy = leds(0, PANEL_LEDS-1);
  for (int x = 0; x < PANEL_WIDTH; ++x) {
    for (int y = 0; y < PANEL_HEIGHT; ++y) {
      leds[ledxy(x,y)] = panelCopy[ledxy(PANEL_WIDTH-x-1, PANEL_HEIGHT-y-1)];
    }
  }

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
  uint8_t earlyDimming = 0xFF;
  if (earlyRunTime > 0) {
    earlyDimming = 0xFF - 0xFF * earlyRunTime / (float)fadeInTime;
  }
  uint8_t totalBrightness = scale8(globalBrightness, earlyDimming);
  LEDS.setBrightness(totalBrightness);
  // leds.fadeLightBy(0xFF - totalBrightness);
}
