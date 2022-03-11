#define FASTLED_ALLOW_INTERRUPTS 0

// FIXME: workaround for build issue in FastLED when FASTLED_ALLOW_INTERRUPTS is 0
#ifndef MS_COUNTER 
#define MS_COUNTER systick_millis_count
#endif
// end FIXME

#define DEBUG 0
#define WAIT_FOR_SERIAL 0

#include <FastLED.h>

#include "config.h"

#define PANEL_COUNT 2 // would require substantial changes to support anything besides 2

#include "util.h"
#include "PatternManager.h"
#include "AudioManager.h"
#include "controls.h"

DrawingContext ctx;

/* ---------------------- */

FrameCounter fc;
HardwareControls controls;
PatternManager<DrawingContext> patternManager(ctx);

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

static unsigned long fadeInStart = 0;

void DrawModal(int fps, unsigned long durationMillis, std::function<void(float progress)> tick) {
  int delayMillis = 1000/fps;
  unsigned long start = millis();
  unsigned long elapsed = 0;
  do {
    float progress = min(1.0, max(0.0, (millis() - start) / (float)durationMillis));
    tick(progress);
    FastLED.show();
    delay(delayMillis);
    elapsed = millis() - start;
  } while (elapsed < durationMillis);
  tick(1.0);
  FastLED.show();
  FastLED.delay(delayMillis);
}

void buttonSinglePress() {
  patternManager.nextPattern();
}

void buttonDoublePress() {
  patternManager.previousPattern();
}

void buttonLongPress() {
  // Automatic mode welcome
  DrawModal(90, 150, [](float progress) {
    ctx.leds.fadeToBlackBy(20);
  });
  ctx.leds.fill_solid(CRGB::Black);
  DrawModal(90, 1200, [](float progress) {
    ctx.pushStyle();
    ctx.drawStyle.boundsBehavior = DrawStyle::clip;
    for (int p = 0; p < 2; ++p) {
      float centerx = (p+1)*PANEL_WIDTH-PANEL_WIDTH/2-0.5;
      float centery = TOTAL_HEIGHT/2. - 0.5;
      int lineCount = 8;
      float bc = 0.3;
      uint8_t brightness = 0xFF * (progress < bc ? progress/bc : progress > (1.0-bc) ? 1.0 - (progress - (1.0-bc)) / bc : 1.0);
      float rotationTheta = millis() / 1000. * M_PI;
      float radius = PANEL_WIDTH/2-0.5;
      ctx.leds.fadeToBlackBy(50);
      for (int i = 0; i < lineCount; ++i) {
        ctx.line(centerx, centery, centerx + radius * sin(rotationTheta + i / (float)lineCount * 2 * M_PI), 
                                   centery + radius * cos(rotationTheta + i / (float)lineCount * 2 * M_PI), 
                                   CHSV(0xFF * i/(float)lineCount, 0xFF, brightness));
      }
    }
    ctx.popStyle();
  });
  ctx.leds.fill_solid(CRGB::Black);
  patternManager.enableAutomaticMode();
  fadeInStart = millis();
}

void buttonDoubleLongPress() {
  logf("double long press");
}

static uint8_t globalBrightness = 0xFF;
void thumbdial1Change(int val) {
  val = map(val, 1023, 0, 0, 0xFF);
  globalBrightness = val;
}

void thumbdial2Change(int val) {
  float gain = 0.01 + val * 1.4 / 1023;
  logf("set extra gain: %f", gain);
  audioManager.setExtraGain(gain);
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
  logf("begin - waited %0.2fs for Serial", (millis() - setupStart) / 1000.);
#elif DEBUG
  delay(2000);
#endif
  
  randomSeed(lsb_noise(UNCONNECTED_PIN_1, 8 * sizeof(uint32_t)));
  random16_add_entropy(lsb_noise(UNCONNECTED_PIN_2, 8 * sizeof(uint16_t)));

  // FIXME: TypicalSMD5050 color correction produces reds at very low brightness levels, such that fading a white pixel down to black produces red at the end.
  // This is very undesirable, so I'd rather use uncorrected colors .setCorrection((UncorrectedColor).
  //.setCorrection(TypicalSMD5050);
  FastLED.addLeds<PANEL_COUNT, WS2812B, DATA_PIN_1, GRB>(ctx.leds, PANEL_LEDS);

  patternManager.ctx = ctx;
  patternManager.setup();

  fc.tick();
  SPSTButton *button = controls.addButton(BUTTON_PIN);
  button->onSinglePress(&buttonSinglePress);
  button->onDoublePress(&buttonDoublePress);
  button->onLongPress(&buttonLongPress);
  button->onDoubleLongPress(&buttonDoubleLongPress);

  AnalogDial *brightnessDial = controls.addAnalogDial(THUMBDIAL1_PIN);
  brightnessDial->onChange(&thumbdial1Change);

  AnalogDial *audioGainDial = controls.addAnalogDial(THUMBDIAL2_PIN);
  audioGainDial->onChange(&thumbdial2Change);
  
  controls.update();

  setupDoneTime = millis();
  fadeInStart = setupDoneTime;
}

void serialTimeoutIndicator() {
  ctx.leds.fill_solid(CRGB::Black);
  if ((millis() - setupDoneTime) % 250 < 100) {
    for (int p = 0; p < PANEL_COUNT; ++p) {
      ctx.line(p*PANEL_WIDTH, 0, p*PANEL_WIDTH, PANEL_HEIGHT-1, CRGB::Red);
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
  panelCopy = ctx.leds(0, PANEL_LEDS-1);
  for (int x = 0; x < PANEL_WIDTH; ++x) {
    for (int y = 0; y < PANEL_HEIGHT; ++y) {
      ctx.leds[ledxy(x,y)] = panelCopy[ledxy(PANEL_WIDTH-x-1, PANEL_HEIGHT-y-1)];
    }
  }

  FastLED.show();
  
  fc.tick();
  fc.clampToFramerate(90);
  controls.update();
}

void applyBrightnessSettings() {
  const long fadeInTime = 2000;

  uint8_t fadeInDimming = 0xFF;
  if (fadeInStart != 0) {
    long earlyRunTime = fadeInStart + fadeInTime - millis();
    if (earlyRunTime > 0) {
      fadeInDimming = 0xFF - 0xFF * earlyRunTime / (float)fadeInTime;
    }
    if (millis() - fadeInStart > fadeInTime) {
      fadeInStart = 0;
    }
  }
  uint8_t totalBrightness = scale8(globalBrightness, fadeInDimming);
  LEDS.setBrightness(totalBrightness);
}
