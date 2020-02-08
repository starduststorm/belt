#ifndef PATTERN_H
#define PATTERN_H

#include <FastLED.h>
#include <Audio.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "util.h"
#include "palettes.h"

class Pattern {
  protected:
    long startTime = -1;
    long stopTime = -1;
    Pattern *subPattern = NULL;

    virtual void stopCompleted() {
      if (!readyToStop()) {
        logf("WARNING: stopped %s before subPattern was stopped", description());
      }
      logf("Stopped %s", description());
      stopTime = -1;
      startTime = -1;
      if (subPattern) {
        subPattern->stop();
        delete subPattern;
        subPattern = NULL;
      }
    }

    virtual Pattern *makeSubPattern() {
      return NULL;
    }

    virtual bool readyToStop() {
      return subPattern == NULL || subPattern->isStopped();
    }

  public:
    virtual ~Pattern() { }

    virtual bool wantsToRun() {
      // for idle patterns that require microphone input and may opt not to run if there is no sound
      return true;
    }

    void start() {
      if (isStopping()) {
        stopCompleted();
      }
      logf("Starting %s", description());
      startTime = millis();
      stopTime = -1;
      setup();
      subPattern = makeSubPattern();
      if (subPattern) {
        subPattern->start();
      }
    }

    void loop(CRGBArray<NUM_LEDS> &leds) {
      update(leds);
      if (subPattern) {
        subPattern->update(leds);
      }
    }

    virtual void setup() { }

    virtual bool wantsToIdleStop() {
      return true;
    }

    virtual void lazyStop() {
      if (isRunning()) {
        logf("Stopping %s", description());
        stopTime = millis();
      }
      if (subPattern) {
        subPattern->lazyStop();
      }
    }

    void stop() {
      if (subPattern) {
        subPattern->stop();
      }
      stopCompleted();
    }

    bool isRunning() {
      return startTime != -1 && isStopping() == false;
    }

    bool isStopped() {
      return !isRunning() && !isStopping();
    }

    long runTime() {
      return startTime == -1 ? 0 : millis() - startTime;
    }

    bool isStopping() {
      return stopTime != -1;
    }

    virtual void update(CRGBArray<NUM_LEDS> &leds) = 0;
    virtual const char *description() = 0;

    // Sub patterns (for pattern mixing)
    void setSubPattern(Pattern *pattern) {
      subPattern = pattern;
      if (isRunning()) {
        subPattern->start();
      }
    }
};


/* --------------------------- */


class Bits : public Pattern {
    enum BitColor {
      monotone, fromPalette, mix, white, pink
    };
    typedef struct _BitsPreset {
      unsigned int maxBits, bitLifespan, updateInterval, fadedown;
      BitColor color;
    } BitsPreset;



// FIXME: eliminate "updateInterval" and instead give bits a velocity in terms of px/s, optionally have them fade-up the next pixel.



    BitsPreset presets[1] = {
        { .maxBits = 10, .bitLifespan = 3000, .updateInterval = 25, .fadedown = 45, .color = fromPalette}, // dots enhancer
//      { .maxBits = 15, .bitLifespan = 3000, .updateInterval = 35, .fadedown = 15, .color = white}, // dots enhancer
//      { .maxBits = 15, .bitLifespan = 3000, .updateInterval = 45, .fadedown = 15, .color = fromPalette}, // dots enhancer
//      // little too frenetic, use as trigger patterns?
//      //      { .maxBits = 10, .bitLifespan = 3000, .updateInterval = 0, .fadedown = 20, .color = monotone }, // party streamers
//      //      { .maxBits = 10, .bitLifespan = 3000, .updateInterval = 0, .fadedown = 20, .color = mix }, // multi-color party streamers
//      { .maxBits = 30, .bitLifespan = 3000, .updateInterval = 1, .fadedown = 30, .color = pink}, // pink triangle
//      { .maxBits = 50, .bitLifespan = 3000, .updateInterval = 16, .fadedown = 15, .color = monotone }, // chill streamers
//      { .maxBits = 50, .bitLifespan = 3000, .updateInterval = 16, .fadedown = 15, .color = fromPalette}, // palette chill streamers
//      { .maxBits = 100, .bitLifespan = 3000, .updateInterval = 16, .fadedown = 40, .color = monotone }, // moving dots
//      { .maxBits = 140, .bitLifespan = 3000, .updateInterval = 350, .fadedown = 15, .color = monotone }, // OG bits pattern
//      { .maxBits = 12, .bitLifespan = 3000, .updateInterval = 4, .fadedown = 70, .color = monotone }, // chase
    };

    class Bit {
        unsigned long birthdate;
      public:
        unsigned int x;
        unsigned int y;
        int dy;
        int dx;
        bool alive;
        unsigned long lastTick;
        CRGB color;
        Bit(CRGB color) {
          reset(color);
        }
        void reset(CRGB color) {
          birthdate = millis();
          alive = true;
          x = random8() % PANEL_WIDTH * PANEL_COUNT;
          y = random8() % PANEL_HEIGHT;
          if (false && random(2)) {
            dy = random(2) ? -1 : 1;
            dx = 0;
          } else {
            dy = 0;
            dx = random(2) ? -1 : 1;
          }
          this->color = color;
        }
        unsigned int age() {
          return millis() - birthdate;
        }
        fract8 ageBrightness() {
          // FIXME: assumes 3000ms lifespan
          float theAge = age();
          if (theAge < 500) {
            return theAge * 0xFF / 500;
          } else if (theAge > 2500) {
            return (3000 - theAge) * 0xFF / 500;
          }
          return 0xFF;
        }
        void tick() {
          x = mod_wrap(x+dx, PANEL_WIDTH);
          y = mod_wrap(y+dy, PANEL_HEIGHT);
          lastTick = millis();
        }
    };

    Bit *bits;
    unsigned int numBits;
    unsigned long lastBitCreation;
    BitsPreset preset;
    uint8_t constPreset;

    CRGB color;
    CRGBPalette16 palette;
  public:
    Bits(int constPreset = -1) {
      this->constPreset = constPreset;
    }
  private:

    CRGB getBitColor() {
      switch (preset.color) {
        case monotone:
          return color; break;
        case fromPalette:
          return ColorFromPalette(palette, random8()); break;
        case mix:
          return CHSV(random8(), random8(200, 255), 255); break;
        case white:
          return CRGB::White;
        case pink:
        default:
          return CRGB::DeepPink;
      }
    }

    void setup() {
      uint8_t pick;
      if (constPreset != -1 && (uint16_t)constPreset < ARRAY_SIZE(presets)) {
        pick = constPreset;
        logf("Using const Bits preset %u", pick);
      } else {
        pick = random8(ARRAY_SIZE(presets));
        logf("  picked Bits preset %u", pick);
      }
      preset = presets[pick];
      
      if (preset.color == fromPalette) {
        int paletteChoice = random16(ARRAY_SIZE(gGradientPalettes));
        logf("  picked palette %i", paletteChoice);
        palette = gGradientPalettes[paletteChoice ];
      }
      
      // for monotone
      color = CHSV(random8(), random8(8) == 0 ? 0 : random8(200, 255), 255);

      bits = (Bit *)calloc(preset.maxBits, sizeof(Bit));
      numBits = 0;
      lastBitCreation = 0;
    }

    void update(CRGBArray<NUM_LEDS> &leds) {
      unsigned long mils = millis();
      bool hasAliveBit = false;
      for (unsigned int i = 0; i < numBits; ++i) {
        Bit *bit = &bits[i];
        if (bit->age() > preset.bitLifespan) {
          bit->alive = false;
        }
        if (bit->alive) {
          leds[ledrc(bit->x, bit->y)] = blend(CRGB::Black, bit->color, bit->ageBrightness());
          if (mils - bit->lastTick > preset.updateInterval) {
            bit->tick();
          }
          hasAliveBit = true;
        } else if (!isStopping()) {
          bit->reset(getBitColor());
          hasAliveBit = true;
        }
      }
      if (isRunning() && numBits < preset.maxBits && mils - lastBitCreation > preset.bitLifespan / preset.maxBits) {
        bits[numBits++] = Bit(getBitColor());
        lastBitCreation = mils;
      }
      if (!isStopping()) {
        leds.fadeToBlackBy(preset.fadedown);
      } else if (!hasAliveBit) {
        stopCompleted();
      }
    }

    void stopCompleted() {
      Pattern::stopCompleted();
      if (bits) {
        free(bits);
        bits = NULL;
        numBits = 0;
      }
    }

    const char *description() {
      return "Bits pattern";
    }
};

/* ------------------- */

class Motion : public Pattern {
private:
  Adafruit_BNO055 bno;
public:
  Motion() {
    bno = Adafruit_BNO055(55, 0x28);
    
    bool hasBNO = bno.begin();
    logf("Has BNO senror: %s", (hasBNO ? "yes" : "no"));
    bno.setExtCrystalUse(true);
  }

  bool wantsToIdleStop() {
    return true;
  }
  
  void setup() {
    bno.enterNormalMode();
  }

  void stopCompleted() {
    bno.enterSuspendMode();
  }
  
  void update(CRGBArray<NUM_LEDS> &leds) {

    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
//    bno.getCalibration(&system, &gyro, &accel, &mag);
//    logf("calibration level: %i", system);
    
    /* Get a new sensor event */ 
    sensors_event_t event; 
    bno.getEvent(&event);
    
//    /* Display the floating point data */
//    Serial.print("X: ");
//    Serial.print(event.orientation.x, 4);
//    Serial.print("\tY: ");
//    Serial.print(event.orientation.y, 4);
//    Serial.print("\tZ: ");
//    Serial.print(event.orientation.z, 4);
//    Serial.println("");


    //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
    //  sensors_event_t orientationData , angVelocityData , linearAccelData;
    //  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    //  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    //  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    //
    //  printEvent(&orientationData);
    //  printEvent(&angVelocityData);
    //  printEvent(&linearAccelData);
    //
    //  int8_t boardTemp = bno.getTemp();
    //  Serial.print(F("temperature: "));
    //  Serial.println(boardTemp);

    for (int x = 0; x < PANEL_WIDTH * PANEL_COUNT; ++x) {
      for (int y = 0; y < PANEL_HEIGHT; ++y) {
        leds[ledrc(x,y)] = CHSV(sin8(5 * x + 2 * event.orientation.x), sin8(5 * y + 2* event.orientation.y), sin8(5 * x + 5 * y + 4 * event.orientation.z));
//          leds[ledrc(x,y)] = CRGB(sin8(15 * x + event.orientation.x), sin8(15 * y + event.orientation.y), sin8(5 *x + 5 * y + 5 * event.orientation.z));
      }
    }
  }
  
  const char *description() {
    return "Motion";
  }

  void printEvent(sensors_event_t* event) {
    Serial.println();
    Serial.print(event->type);
    double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
    if (event->type == SENSOR_TYPE_ACCELEROMETER) {
      x = event->acceleration.x;
      y = event->acceleration.y;
      z = event->acceleration.z;
    }
    else if (event->type == SENSOR_TYPE_ORIENTATION) {
      x = event->orientation.x;
      y = event->orientation.y;
      z = event->orientation.z;
    }
    else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
      x = event->magnetic.x;
      y = event->magnetic.y;
      z = event->magnetic.z;
    }
    else if ((event->type == SENSOR_TYPE_GYROSCOPE) || (event->type == SENSOR_TYPE_ROTATION_VECTOR)) {
      x = event->gyro.x;
      y = event->gyro.y;
      z = event->gyro.z;
    }
  
    Serial.print(": x= ");
    Serial.print(x);
    Serial.print(" | y= ");
    Serial.print(y);
    Serial.print(" | z= ");
    Serial.println(z);
  }
};

/* ------------------- */

class Droplets : public Pattern {
  private:
    unsigned long lastDrop;
    unsigned long lastFlow;
    CRGB cs[NUM_LEDS];
    CRGBPalette16 palette;
    bool usePalette;

    void setup() {
      usePalette = (random(3) > 0);
      if (usePalette) {
        palette = gGradientPalettes[random16(gGradientPaletteCount)];
      }
    }
    
    void update(CRGBArray<NUM_LEDS> &leds) {
      const unsigned int dropInterval = 80;
      const unsigned int flowInterval = 30;
      const float kFlow = 0.2;
      const float kEff = 0.99;
      const int minLoss = 1;

      unsigned long mils = millis();
      if (mils - lastDrop > dropInterval) {
        int center = random16(NUM_LEDS);
        CRGB color;
        if (usePalette) {
          color = ColorFromPalette(palette, random8());
        } else {
          color = CHSV(random8(), 255, 255);
        }
        for (int i = -2; i < 3; ++i) {
          leds[mod_wrap(center + i, NUM_LEDS)] = color;
        }
        lastDrop = mils;
      }
      if (mils - lastFlow > flowInterval) {
        for (int i = 0; i < NUM_LEDS; ++i) {
          cs[i] = leds[i];
        }
        for (int i = 0; i < NUM_LEDS; ++i) {
          int i2 = (i + 1) % NUM_LEDS;
          // calculate flows from og leds, set in scratch
          CRGB led1 = leds[i];
          CRGB led2 = leds[i2];
          for (uint8_t sp = 0; sp < 3; ++sp) { // each subpixel
            uint8_t *refSp = NULL;
            uint8_t *srcSp = NULL;
            uint8_t *dstSp = NULL;
            if (led1[sp] < led2[sp]) {
              refSp = &led2[sp];
              srcSp = &cs[i2][sp];
              dstSp = &cs[i][sp];
            } else if (led1[sp] > led2[sp] ) {
              refSp = &led1[sp];
              srcSp = &cs[i][sp];
              dstSp = &cs[i2][sp];
            }
            if (srcSp && dstSp) {
              uint8_t flow = min(*srcSp, min((int)(kFlow * *refSp), 0xFF - *dstSp));
              *dstSp += kEff * flow;
              if (*srcSp > flow && *srcSp > minLoss) {
                *srcSp -= max(minLoss, flow);
              } else {
                *srcSp = 0;
              }
            }
          }
        }
        for (int i = 0; i < NUM_LEDS; ++i) {
          leds[i] = cs[i];
        }
        lastFlow  = mils;
      }
      if (isStopping()) {
        stopCompleted();
      }
    }

    const char *description() {
      return "Droplets";
    }
};

/* ------------------- */

#define SECONDS_PER_PALETTE 20
uint8_t gCurrentPaletteNumber = 0;
CRGBPalette16 gCurrentPalette( CRGB::Black);
CRGBPalette16 gTargetPalette( gGradientPalettes[0] );

class SmoothPalettes : public Pattern {
    void setup() {
      gTargetPalette = gGradientPalettes[random16(gGradientPaletteCount)];
    }
    void update(CRGBArray<NUM_LEDS> &leds) {
      EVERY_N_MILLISECONDS(20) {
        draw(leds);
      }
      if (isStopping()) {
        stopCompleted();
      }
    }

    void draw(CRGBArray<NUM_LEDS> &leds) {
      // ColorWavesWithPalettes
      // Animated shifting color waves, with several cross-fading color palettes.
      // by Mark Kriegsman, August 2015
      CRGBPalette16& palette = gCurrentPalette;
      EVERY_N_SECONDS( SECONDS_PER_PALETTE ) {
        gCurrentPaletteNumber = addmod8( gCurrentPaletteNumber, random8(16), gGradientPaletteCount);
        gTargetPalette = gGradientPalettes[ gCurrentPaletteNumber ];
      }

      EVERY_N_MILLISECONDS(40) {
        nblendPaletteTowardPalette( gCurrentPalette, gTargetPalette, 16);
      }

      uint16_t numleds = NUM_LEDS;
      static uint16_t sPseudotime = 0;
      static uint16_t sLastMillis = 0;
      static uint16_t sHue16 = 0;

      //      uint8_t sat8 = beatsin88( 87, 220, 250);
      uint8_t brightdepth = beatsin88( 341, 96, 224);
      uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
      uint8_t msmultiplier = beatsin88(147, 23, 60);

      uint16_t hue16 = sHue16;//gHue * 256;
      uint16_t hueinc16 = beatsin88(113, 300, 1500);

      uint16_t ms = millis();
      uint16_t deltams = ms - sLastMillis ;
      sLastMillis  = ms;
      sPseudotime += deltams * msmultiplier;
      sHue16 += deltams * beatsin88( 400, 5, 9);
      uint16_t brightnesstheta16 = sPseudotime;

      for ( uint16_t i = 0 ; i < numleds; i++) {
        hue16 += hueinc16;
        uint8_t hue8 = hue16 / 256;
        uint16_t h16_128 = hue16 >> 7;
        if ( h16_128 & 0x100) {
          hue8 = 255 - (h16_128 >> 1);
        } else {
          hue8 = h16_128 >> 1;
        }

        brightnesstheta16  += brightnessthetainc16;
        uint16_t b16 = sin16( brightnesstheta16  ) + 32768;

        uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
        uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
        bri8 += (255 - brightdepth);

        uint8_t index = hue8;
        //index = triwave8( index);
        index = scale8( index, 240);

        CRGB newcolor = ColorFromPalette( palette, index, bri8);

        uint16_t pixelnumber = i;
        pixelnumber = (numleds - 1) - pixelnumber;

        uint8_t blendAmt = runTime() < 2000 ? runTime() / 15 : 128;
        nblend( leds[pixelnumber], newcolor, blendAmt);
      }
    }
    const char *description() {
      return "Smooth palettes";
    }
};

/* ------------------- */

class RaverPlaid : public Pattern {
  // how many sine wave cycles are squeezed into our n_pixels
  // 24 happens to create nice diagonal stripes on the wall layout
  const int default_freq = 24;
  float freq_r = default_freq;
  float freq_g = default_freq;
  float freq_b = default_freq;
  int mode;

  void setup() {
    if (random8(2) == 0) {
      logf("  Default frequencies\n");
      freq_r = default_freq;
      freq_g = default_freq;
      freq_b = default_freq;
    } else {
      freq_r = random8(18, 30);
      freq_g = random8(18, 30);
      freq_b = random8(18, 30);;
      logf("  frequencies %f, %f, %f\n", freq_r, freq_g, freq_b);
    }
    // 20% chance to cut out each channel
    mode = random8(5);
  }

  void update(CRGBArray<NUM_LEDS> &leds) {
    // Demo code from Open Pixel Control
    // http://github.com/zestyping/openpixelcontrol
    int n_pixels = NUM_LEDS;

// NEEDS TO BE UPDATED with FASTLED INTEGER MATH FOR ADAQUATE FRAMERATE. god it's so hard to hold shift for that long to type in all caps if only there were a better way
// FIXME: and I probably broke this while trying to improve framerate. Got it from 20-45 tho.

    // how many seconds the color sine waves take to shift through a complete cycle
    float speed_r = 7;
    float speed_g = -13;
    float speed_b = 19;
    float t = runTime() / 1000. * 5;
    for (int ii = 0; ii < n_pixels; ++ii) {
        float pct = (ii / (float)n_pixels);
        // diagonal black stripes
        float pct_jittered = fmod_wrap(pct * 77, 37);
//        float blackstripes = beatsin8(pct_jittered, t*0.05, 1, -1.5, 1.5);
        float blackstripes = remap(beatsin8(pct_jittered, 0, 255, 1, t*0.05), 0, 255, -1.5, 1.5);
        float blackstripes_offset = remap(beatsin8(pct_jittered, 0, 255, 0.9, 60), 0, 255, -0.5, 3);
//          util_cos(t, 0.9, 60, -0.5, 3);
        blackstripes = clamp(blackstripes + blackstripes_offset, 0, 1);
        // 3 sine waves for r, g, b which are out of sync with each other
        char r = blackstripes * cos8((t/speed_r + pct*freq_r)*M_PI*2);
        char g = blackstripes * cos8((t/speed_g + pct*freq_g)*M_PI*2);
        char b = blackstripes * cos8((t/speed_b + pct*freq_b)*M_PI*2);

        r = (mode == 2 ? 0 : r);
        g = (mode == 3 ? 0 : g);
        b = (mode == 4 ? 0 : b);

        leds[ii] = CRGB(r,g,b);
    }
    if (isStopping()) {
      stopCompleted();
    }
    // usleep(1. / fps * 1000000);
  }
  const char *description() {
    return "Raver Plaid";
  }
};

/* ------------------- */

#endif
