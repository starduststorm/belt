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

// HACK: Creating/initializing MotionManager (and the BNO) before AudioManager (and the i2s) resolves a crashing issue
// I don't understand why.
MotionManager motionManager;
#if USE_AUDIO
AudioManager audioManager;
#endif

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
          leds[ledxy(bit->x, bit->y)] = blend(CRGB::Black, bit->color, bit->ageBrightness());
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

#define EXTRA_GAIN 200

class Sound: public Pattern, public PaletteRotation<CRGBPalette256> {
private:
  int framesWithoutAudioData = 0;
  float levelsAccum[BAND_COUNT];
  float *levels = NULL;
public:
  Sound() : PaletteRotation(includePalettesWithBlack=false) {
  }
  
  void setup() {
    bzero(levelsAccum, sizeof(levelsAccum));
    audioManager.subscribe();
  }

  void stopCompleted() {
    Pattern::stopCompleted();
    audioManager.unsubscribe();
  }
  
  void update(CRGBArray<NUM_LEDS> &leds) {
    leds.fill_solid(CRGB::Black);
    
    const int bandRunningAvgCount = 4;
    if (audioManager.available()) {
      if (framesWithoutAudioData > 0) {
//          logf("Went %i frames without audio data", framesWithoutAudioData);
        framesWithoutAudioData = 0;
      }
      
      levels = audioManager.getLevels();
      for (int i = 0; i < BAND_COUNT; ++i) {
        levels[i] *= EXTRA_GAIN;
      }
    } else {
      framesWithoutAudioData++;
    }
    if (levels != NULL) {
      for (int x = 0; x < PANEL_WIDTH; ++x) {
        float level = levels[x+2];
        
        levelsAccum[x] = (levelsAccum[x] * bandRunningAvgCount + level) / (float)(bandRunningAvgCount + 1);
        
        // Just draw the same thing to both panels for now
        for (int p = 0; p < PANEL_COUNT; ++p) {
          for (int y = 0; y <= levelsAccum[x] && y < PANEL_HEIGHT; ++y) {
            char bright = min(0xFF, (levelsAccum[x] - y) * 0xFF);
//            CRGB color = CHSV(20 * y, 0xFF, bright); // rainbow            
            CRGB color = getPaletteColor(map(y, 0, PANEL_HEIGHT-1, 0, 0xFF));
            color.nscale8_video(bright);
            leds[ledxy(x + p * PANEL_WIDTH, y)] = color;
          }
        }
      }
    }
    
    EVERY_N_MILLISECONDS(5000) {
      audioManager.printStatistics();
    }
    
    if (isStopping()) {
      stopCompleted();
    }
  }
  
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
    Pattern::stopCompleted();
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

class Bars : public Pattern, public PaletteRotation<CRGBPalette16> {
  const int kSecondsPerPalette = 10;
  const int kBarWidth = 8;
  unsigned numBars = 0;
  
  float *xvelocities = NULL;
  float *xoffsets = NULL;
  
  void setup(DrawingContext &ctx) {
    numBars = ctx.width / kBarWidth / 2 * ctx.height;

    prepareTrackedColors(numBars);
    
    xvelocities = new float[ctx.height];
    xoffsets= new float[ctx.height];

    for (int i = 0 ; i < ctx.height; ++i) {
      xvelocities[i] = random8() / (255.0 * 2) + 0.75;
      xoffsets[i] = random8(8);
    }
  }

  void stopCompleted() {
    Pattern::stopCompleted();
    delete [] xvelocities;
    xvelocities = NULL;
    delete [] xoffsets;
    xoffsets = NULL;

    releaseTrackedColors();
  }
  
  void update(DrawingContext &ctx) {
    ctx.leds.fill_solid(CRGB::Black);

    EVERY_N_MILLISECONDS(20) {
      shiftTrackedColors(1);
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

        CRGB color = getTrackedColor(y * ctx.width / kBarWidth / 2 + x / 2);
        ctx.line(x1, y, x2, y, color);
      }
      xoffsets[y] += xvelocities[y] * frameTime() / 30.0;
      if (xoffsets[y] > ctx.width) {
        xoffsets[y] -= ctx.width;
      }
    }

    ctx.popStyle();

    if (isStopping()) {
      stopCompleted();
    }
  }
  
  const char *description() {
    return "Bars";
  }
};

/* ------------------------------------------------------------------------------------------------------ */

class Oscillators : public Pattern {
  void update(DrawingContext &ctx) {
    for (int index = 0; index < ctx.width * ctx.height; ++index) {
      
      // this looks cool for a good long time
//      int r = 0;//beatsin16(millis() / 15 + 5 * index) >> 8;
//      int g = beatsin16(millis() / 10 + 10 * index) >> 8;
//      int b = 0;//beatsin16(millis() / 5 + 15 * index) >> 8;
//      EVERY_N_MILLISECONDS(10) {
//        logf("b1 = %i, b2 = %i", b1, b2);
//      }
//CRGB color = CRGB(dim8_raw(r), dim8_raw(g), dim8_raw(b));
//      ctx.leds[index] = color;
    }

    for (int x = 0; x < ctx.width; ++x) {
      for (int y = 0; y < ctx.height; ++y) {
        CHSV color = CHSV(100, 0xFF, sin8(millis()/2. + x*40 + beatsin8(30)*y));// + beatsin8(30, 20, 0xFF - 20*7));
        ctx.point(x, y, color);
      }
    }
    if (isStopping()) {
      stopCompleted();
    }
  }
  const char *description() {
    return "Oscillators";
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
    Pattern::stopCompleted();
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
        leds[ledxy(x,y)] = CHSV(sin8(5 * x + 2 * event.orientation.x), sin8(5 * y + 2* event.orientation.y), sin8(5 * x + 5 * y + 4 * event.orientation.z));
//          leds[ledxy(x,y)] = CRGB(sin8(15 * x + event.orientation.x), sin8(15 * y + event.orientation.y), sin8(5 *x + 5 * y + 5 * event.orientation.z));
      }
    }
  }
  
  const char *description() {
    return "RainbowMotion";
  }
};

/* ------------------------------------------------------------------------------------------------------ */

class PixelDust : public Pattern {
private:
  static const unsigned int numParticles = 50;
  Adafruit_PixelDust *pixelDust[PANEL_COUNT];

  const int kSecondsPerPalette = 10;
  CRGBPalette16 currentPalette;
  CRGBPalette16 targetPalette;

  int *dustColorIndexes = NULL;
public:
  PixelDust() {
    for (int i = 0; i < PANEL_COUNT; ++i) {
      pixelDust[i] = new Adafruit_PixelDust(PANEL_WIDTH, PANEL_HEIGHT, numParticles, 0xFF, 101, false);
    }
  }

  void setup() {
    motionManager.subscribe();
    for (int i = 0; i < PANEL_COUNT; ++i) {
      if(!pixelDust[i]->begin()) {
        logf("PixelDust init failed");
      }
      pixelDust[i]->randomize();
    }
        
    assert(dustColorIndexes == NULL, "dustColorIndexes memory not handled properly");
    dustColorIndexes = new int[numParticles];
    resetPalette();
  }

  void resetPalette() {
    currentPalette = paletteManager.randomPalette();
    targetPalette = paletteManager.randomPalette();
    
    for (unsigned i = 0; i < numParticles; ++i) {
      dustColorIndexes[i] = random8();
    }
  }

  void stopCompleted() {
    Pattern::stopCompleted();
    motionManager.unsubscribe();
    
    delete [] dustColorIndexes;
    dustColorIndexes = NULL;
  }

  void update(CRGBArray<NUM_LEDS> &leds) {
    EVERY_N_MILLISECONDS(40) {
      nblendPaletteTowardPalette(currentPalette, targetPalette, 16);
    }
    EVERY_N_SECONDS(kSecondsPerPalette) {
      targetPalette = paletteManager.randomPalette();
    }

    
    dimension_t x, y;
    sensors_event_t accel;
    sensors_event_t linear_accel;
    motionManager.bno.getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    motionManager.bno.getEvent(&linear_accel, Adafruit_BNO055::VECTOR_LINEARACCEL);

    float jerkFactor = 2.0;

    for (int i = 0; i < PANEL_COUNT; ++i) {
      pixelDust[i]->iterate(
                    1 * (accel.acceleration.x + jerkFactor * linear_accel.acceleration.x),
                   -1 * (accel.acceleration.y + jerkFactor * linear_accel.acceleration.y), 
                    1 * (accel.acceleration.z + jerkFactor * linear_accel.acceleration.z));
    }

    leds.fill_solid(CRGB::Black);

    for (int i = 0; i < PANEL_COUNT; ++i) {
      for (unsigned int p = 0; p < numParticles; p++) {
        pixelDust[i]->getPosition(p, &x, &y);
        
        CRGB color = ColorFromPalette(currentPalette, dustColorIndexes[p]);
        while (!color) {
          dustColorIndexes[p]++;
          if (dustColorIndexes[p] > 0xFF) {
            dustColorIndexes[p] -= 0xFF;
          }
          color = ColorFromPalette(currentPalette, dustColorIndexes[p]);
        }
        leds[ledxy(x + i * PANEL_WIDTH, y)] = color;
      }
    }

    if (isStopping()) {
      stopCompleted();
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

#endif
