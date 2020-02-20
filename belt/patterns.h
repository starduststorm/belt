#ifndef PATTERN_H
#define PATTERN_H

#include <FastLED.h>
#include <Audio.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_PixelDust.h>
#include <utility/imumaths.h>

#include "util.h"
#include "palettes.h"
#include "AudioManager.h"
#include "MotionManager.h"
#include "drawing.h"

#if USE_AUDIO
AudioManager audioManager;
#endif
MotionManager motionManager;

class Pattern {
  protected:
    long startTime = -1;
    long stopTime = -1;
    long lastUpdateTime = -1;
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

    void start(DrawingContext &ctx) {
      if (isStopping()) {
        stopCompleted();
      }
      logf("Starting %s", description());
      startTime = millis();
      stopTime = -1;
      setup(ctx);
      setup();
      subPattern = makeSubPattern();
      if (subPattern) {
        subPattern->start(ctx);
      }
    }

    void loop(DrawingContext &ctx) {
      update(ctx.leds);
      update(ctx);
      if (subPattern) {
        subPattern->update(ctx.leds);
        subPattern->update(ctx);
      }
      lastUpdateTime = millis();
    }

    // Both of these are called
    virtual void setup(DrawingContext &ctx) { }
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

    long frameTime() {
      return (lastUpdateTime == -1 ? 0 : millis() - lastUpdateTime);
    }

    bool isStopping() {
      return stopTime != -1;
    }

    // One of the two of these must be implemented. Both will be called.
    virtual void update(CRGBArray<NUM_LEDS> &leds) { }
    virtual void update(DrawingContext &ctx) { }
    
    virtual const char *description() = 0;
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
        palette = gGradientPalettes[paletteChoice];
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

/* ------------------------------------------------------------------------------------------------------ */


// FIXME: look into just grabbing lots of data and pushing it through map_data_into_colors_through_palette, using data stream from microphone or motion




#if USE_AUDIO

class Sound: public Pattern {
  private:

    CRGBPalette16 palette;
    bool usePalette;
    float bands[BAND_COUNT];

    void setup() {
      usePalette = (random(3) > 0);
      if (usePalette) {
        palette = gGradientPalettes[random16(gGradientPaletteCount)];
      }
      bzero(bands, sizeof(bands));
      audioManager.subscribe();
    }

    void stopCompleted() {
      audioManager.unsubscribe();
    }
    
    void update(CRGBArray<NUM_LEDS> &leds) {
      const int bandRunningAvgCount = 5;
      if (audioManager.available()) {
        leds.fill_solid(CRGB::Black);
          
        float *levels = audioManager.getLevels();

        for (int x = 0; x < PANEL_WIDTH; ++x) {
          float level = levels[x+2];
      
          level *= 200;

          bands[x] = (bands[x] * bandRunningAvgCount + level) / (float)(bandRunningAvgCount + 1);

          for (int y = 0; y <= bands[x] && y < PANEL_HEIGHT; ++y) {
            char bright = min(0xFF, (bands[x] - y) * 0xFF);
            leds[ledrc(x, y)] = CHSV(20 * y, 0xFF, bright);
          }
        }
      }

    EVERY_N_MILLISECONDS(1000) {
      audioManager.printStatistics();
    }
      
      if (isStopping()) {
        stopCompleted();
      }
    }
public:
    const char *description() {
      return "Sound";
    }
};

#endif

/* ------------------------------------------------------------------------------------------------------ */

class Motion : public Pattern {
  float kMotionThreshold = 2.0;
private:
  class MotionBlob {
    int y;
    int x;
    float xfact;
  public:
    CRGB color;
    void reset(int x, int y, CRGB color) {
      this->x = x;
      this->y = y;
      this->color = color;
      xfact = random8() / 255. + 0.5;
    }
    
    void update(DrawingContext &ctx, float theta, float dtheta) {
      // TODO: this needs to be tuned to compensate for the total angle the leds will occupy in the circle
      int x_offset = xfact * map(theta, -180, 180, 0, ctx.width);
      
      int brightness = min(0xFF, 100 * (dtheta - 1));
      CRGB dimmedColor = color;
      dimmedColor.nscale8_video(brightness);

      int px =  (x + x_offset) % ctx.width;
      int px2 = (x + x_offset + 1) % ctx.width;

      ctx.point(px,  y,   dimmedColor);
      ctx.point(px2, y,   dimmedColor);
      ctx.point(px,  y+1, dimmedColor);
      ctx.point(px2, y+1, dimmedColor);
    }
  };
  
public:
  const int kBlobCount = 16;
  const int kBlobsResetTimeout = 5000;
  int prevOrientation;
  float rotationVelocity = 0;
  const float rotationSamples = 5;
  MotionBlob *blobs;
  CRGBPalette16 palette;
  long lastDisplay = -kBlobsResetTimeout;

  void resetBlobs(DrawingContext &ctx) {
    // space out the blobs so they don't overlap
    const int ybuckets = 2;
    const int xbuckets = kBlobCount / ybuckets;

    const int xbucketSize = ctx.width / xbuckets;
    const int ybucketSize = ctx.height / ybuckets;
    
    palette = paletteManager.randomPalette();
    for (int i = 0; i < kBlobCount; ++i) {
      int xbucketStart = (i * xbucketSize) % ctx.width;
      int ybucketStart = ybucketSize * ((i * xbucketSize) / ctx.width);
      
      int x = xbucketStart + random8(xbucketSize);
      int y = ybucketStart + random8(ybucketSize - 1);
      CRGB color = ColorFromPalette(palette, random8());
      
      blobs[i].reset(x, y, color);
    }
  }
  
  void setup() {
    motionManager.subscribe();
    assert(blobs == NULL, "MotionBlobs array not nulled");
    blobs = new MotionBlob[kBlobCount];
  }

  void stopCompleted() {
    motionManager.unsubscribe();
    delete [] blobs;
    blobs = NULL;
  }

  void update(DrawingContext &ctx) {
    ctx.leds.fadeToBlackBy(25);

    if (lastDisplay != -1 && millis() - lastDisplay > 5000) {
      resetBlobs(ctx);
      lastDisplay = -1;
    }
    
    sensors_event_t event; 
    motionManager.getEvent(&event);
    
//    logf("orientation = (%f, %f, %f)", event.orientation.x, event.orientation.y, event.orientation.z);

    float orientation = event.orientation.z;
    rotationVelocity = (rotationSamples * rotationVelocity + prevOrientation - orientation) / (rotationSamples + 1);
    prevOrientation = orientation;

    float rotationSpeed = fabs(rotationVelocity);

    if (rotationSpeed > kMotionThreshold) {
      ctx.pushStyle();
      ctx.blendMode(brighten);
      for (int i = 0; i < kBlobCount; ++i) {
        MotionBlob *blob = &blobs[i];
        blob->update(ctx, orientation, rotationSpeed);
      }
      ctx.popStyle();
      lastDisplay = millis();
    }
  }
  
  bool wantsToIdleStop() {
    return true;
  }

  const char *description() {
    return "Motion";
  }
};

/* ------------------------------------------------------------------------------------------------------ */


// idea: a pattern that can handle motion, sound, or be idle
// this is the idle version. could feed audio into it to make the bars pulse in brightness, or feed motion to make them move

class Bars : public Pattern {
  const int kSecondsPerPalette = 10;
  const int kBarWidth = 8;
   int numBars = 0;
  
  int *colorIndexes = NULL;
  float *xvelocities = NULL;
  float *xoffsets = NULL;
  
  CRGBPalette16 currentPalette;
  CRGBPalette16 targetPalette;
  
  void setup(DrawingContext &ctx) {
    currentPalette = paletteManager.randomPalette();
    targetPalette = paletteManager.randomPalette();
    
    assert(colorIndexes == NULL, "Color indexes array not nulled");
    numBars = ctx.width / kBarWidth / 2 * ctx.height;
    colorIndexes = new int[numBars];
    xvelocities = new float[ctx.height];
    xoffsets= new float[ctx.height];

    for (unsigned i = 0; i < numBars; ++i) {
      colorIndexes[i] = random8();
    }

    for (int i = 0 ; i < ctx.height; ++i) {
      xvelocities[i] = random8() / (255.0 * 2) + 0.75;
      xoffsets[i] = random8(8);
    }
  }

  void stopCompleted() {
    delete [] colorIndexes;
    colorIndexes = NULL;
    delete [] xvelocities;
    xvelocities = NULL;
    delete [] xoffsets;
    xoffsets = NULL;
  }
  
  void update(DrawingContext &ctx) {
    FastLED.clear();
    
    EVERY_N_SECONDS(kSecondsPerPalette) {
      targetPalette = paletteManager.randomPalette();
    }
    
    EVERY_N_MILLISECONDS(40) {
      nblendPaletteTowardPalette(currentPalette, targetPalette, 16);
    }
   
    EVERY_N_MILLISECONDS(20) {
      for (unsigned i = 0; i < numBars; ++i) {
        colorIndexes[i] = addmod8(colorIndexes[i], 1, 0xFF);
      }
    }
    
    ctx.pushStyle();
    ctx.drawStyle.wrap = true;
    
    for (int y = 0; y < ctx.height; ++y) {
      for (int x = 0; x < ctx.width / kBarWidth; ++x) {
        if (addmod8(x, y, 2) == 0) {
          continue;
        }
        int row_startx = (y & 1 ? xoffsets[y]: -xoffsets[y]);
        
        int x1 = row_startx + x*kBarWidth;
        int x2 = row_startx + (x+1)*kBarWidth - 1;

        CRGB color = ColorFromPalette(currentPalette, colorIndexes[y * ctx.width / kBarWidth / 2 + x / 2]);
        ctx.line(x1, y, x2, y, color);
      }
      xoffsets[y] += xvelocities[y] * frameTime() / 30.0;
      if (xoffsets[y] > ctx.width) {
        xoffsets[y] -= ctx.width;
      }
    }

    ctx.popStyle();
  }
  
  const char *description() {
    return "Bars";
  }
};

/* ------------------------------------------------------------------------------------------------------ */

class RainbowMotion : public Pattern {
  bool wantsToIdleStop() {
    return true;
  }
  
  void setup() {
    motionManager.subscribe();
  }

  void stopCompleted() {
    motionManager.unsubscribe();
  }

  void update(CRGBArray<NUM_LEDS> &leds) {
    sensors_event_t event; 
    motionManager.getEvent(&event);
    
    logf("X: %0.4f, Y: %0.4f, Z: %0.4f", event.orientation.x, event.orientation.y, event.orientation.z);
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
    //  motionManager.printEvent(&orientationData);
    //  motionManager.printEvent(&angVelocityData);
    //  motionManager.printEvent(&linearAccelData);
    //

    for (int x = 0; x < PANEL_WIDTH * PANEL_COUNT; ++x) {
      for (int y = 0; y < PANEL_HEIGHT; ++y) {
        leds[ledrc(x,y)] = CHSV(sin8(5 * x + 2 * event.orientation.x), sin8(5 * y + 2* event.orientation.y), sin8(5 * x + 5 * y + 4 * event.orientation.z));
//          leds[ledrc(x,y)] = CRGB(sin8(15 * x + event.orientation.x), sin8(15 * y + event.orientation.y), sin8(5 *x + 5 * y + 5 * event.orientation.z));
      }
    }
  }
  
  const char *description() {
    return "RainbowMotion";
  }
};


/*
 *    A palette blend could look cool here but I'd need to handle black pixels
 *    
 *     EVERY_N_SECONDS( SECONDS_PER_PALETTE ) {
        gCurrentPaletteNumber = addmod8( gCurrentPaletteNumber, random8(16), gGradientPaletteCount);
        gTargetPalette = gGradientPalettes[ gCurrentPaletteNumber ];
      }

      EVERY_N_MILLISECONDS(40) {
        nblendPaletteTowardPalette( gCurrentPalette, gTargetPalette, 16);
      }
 */
class PixelDust : public Pattern {
private:
  static const unsigned int numParticles = 50;
  Adafruit_PixelDust *pixelDust;
  CRGBPalette16 palette;
  CRGB *dustColors = NULL;
public:
  PixelDust() {
    pixelDust = new Adafruit_PixelDust(PANEL_WIDTH, PANEL_HEIGHT, numParticles, 0xFF, 101, false);
  }

  void setup() {
    motionManager.subscribe();
    if(!pixelDust->begin()) {
      logf("PixelDust init failed");
    }
  
    pixelDust->randomize();
    logf("picking palette");
    
    if (dustColors) {
      delete [] dustColors;
    }
    dustColors = new CRGB[numParticles];
    resetPalette();
  }

  void resetPalette() {
    palette = paletteManager.randomPalette();

    for (unsigned i = 0; i < numParticles; ++i) {
      CRGB color = CRGB::Black;
      do {
        color = ColorFromPalette(palette, random8());
        dustColors[i] = color;
      } while (!color);
    }
  }

  void stopCompleted() {
    motionManager.unsubscribe();
    
    delete [] dustColors;
    dustColors = NULL;
  }

  void update(CRGBArray<NUM_LEDS> &leds) {
    dimension_t x, y;
    sensors_event_t accel;
    sensors_event_t linear_accel;
    motionManager.bno.getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    motionManager.bno.getEvent(&linear_accel, Adafruit_BNO055::VECTOR_LINEARACCEL);

    float jerkFactor = 2.0;
    
    pixelDust->iterate(
                  1 * (accel.acceleration.x + jerkFactor * linear_accel.acceleration.x),
                 -1 * (accel.acceleration.y + jerkFactor * linear_accel.acceleration.y), 
                  1 * (accel.acceleration.z + jerkFactor * linear_accel.acceleration.z));

    FastLED.clear();

    for (unsigned int i=0; i<numParticles; i++) {
      pixelDust->getPosition(i, &x, &y);
      leds[ledrc(x, y)] = dustColors[i];
    }

    EVERY_N_SECONDS(10) {
      resetPalette();
    }
  }

  const char *description() {
    return "PixelDust";
  }
};


/* ------------------------------------------------------------------------------------------------------ */

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
