#ifndef PATTERN_H
#define PATTERN_H

#include <FastLED.h>
#include <Audio.h>
#include <map>

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
AudioManager audioManager;

class Pattern {
private:  
  long startTime = -1;
  long stopTime = -1;
  long lastUpdateTime = -1;
public:
  DrawingContext ctx;
  virtual ~Pattern() { }

  void start() {
    logf("Starting %s", description());
    startTime = millis();
    stopTime = -1;
    ctx.leds.fill_solid(CRGB::Black);
    setup();
  }

  void loop() {
    update();
    lastUpdateTime = millis();
  }

  virtual bool wantsToIdleStop() {
    return true;
  }

  virtual bool wantsToRun() {
    // for idle patterns that require microphone input and may opt not to run if there is no sound
    return true;
  }

  virtual void setup() { }

  void stop() {
    logf("Stopping %s", description());
    startTime = -1;
  }

  virtual void update() { }
  
  virtual const char *description() = 0;

public:
  bool isRunning() {
    return startTime != -1;
  }

  long runTime() {
    return startTime == -1 ? 0 : millis() - startTime;
  }

  long frameTime() {
    return (lastUpdateTime == -1 ? 0 : millis() - lastUpdateTime);
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
          x = random8() % TOTAL_WIDTH;
          y = random8() % TOTAL_HEIGHT;
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
          x = mod_wrap(x+dx, TOTAL_WIDTH);
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
    ~Bits() {
      free(bits);
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

    void update() {
      CRGBArray<NUM_LEDS> leds = ctx.leds;
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
        } else if (isRunning()) {
          bit->reset(getBitColor());
          hasAliveBit = true;
        }
      }
      if (isRunning() && numBits < preset.maxBits && mils - lastBitCreation > preset.bitLifespan / preset.maxBits) {
        bits[numBits++] = Bit(getBitColor());
        lastBitCreation = mils;
      }
      if (isRunning()) {
        // TODO: not needed after global pattern crossfade
        leds.fadeToBlackBy(preset.fadedown);
      } else if (!hasAliveBit) {
        // done fading out
      }
    }

    const char *description() {
      return "Bits pattern";
    }
};

/* ------------------------------------------------------------------------------------------------------ */


// FIXME: look into just grabbing lots of data and pushing it through map_data_into_colors_through_palette, using data stream from microphone or motion


#define EXTRA_GAIN 200

class SpectrumAnalyzer: public Pattern, public PaletteRotation<CRGBPalette256>, public FFTProcessing {
private:
  static const int fftBinCount = PANEL_WIDTH;
  int framesWithoutAudioData = 0;
  const float *levels = NULL;
  bool upsideDown = false;
public:
  SpectrumAnalyzer() : PaletteRotation(minBrightness=10), FFTProcessing(&audioManager, fftBinCount) {
    motionManager.subscribe();
  }

  ~SpectrumAnalyzer() {
    motionManager.unsubscribe();
  }
  
  void setup() {
    upsideDown = (random8(2) == 0);
  }
  
  void update() {
    CRGBArray<NUM_LEDS> leds = ctx.leds;
    leds.fill_solid(CRGB::Black);
    
    if (fftAvailable()) {
      if (framesWithoutAudioData > 0) {
//          logf("Went %i frames without audio data", framesWithoutAudioData);
        framesWithoutAudioData = 0;
      }
      
      levels = fftLevels(4);
    } else {
      framesWithoutAudioData++;
    }

    if (levels != NULL) {
      for (int x = 0; x < min(fftBinCount, PANEL_WIDTH); ++x) {
        float level = levels[x] * EXTRA_GAIN;
        // Just draw the same thing to both panels for now
        for (int p = 0; p < PANEL_COUNT; ++p) {
          for (int y = 0; y <= level && y < PANEL_HEIGHT; ++y) {
            char bright = min(0xFF, (level - y) * 0xFF);
//            CRGB color = CHSV(20 * y, 0xFF, bright); // rainbow            
            CRGB color = getPaletteColor(map(y, 0, PANEL_HEIGHT-1, 0, 0xFF));
            color.nscale8_video(bright);
            leds[ledxy(x + p * PANEL_WIDTH, (upsideDown ? TOTAL_HEIGHT - y - 1 : y))] = color;
          }
        }
      }
    }

    sensors_event_t event;
    motionManager.getEvent(&event);
    float xOffset = event.orientation.x * TOTAL_WIDTH / 360;
    ctx.shift_buffer(-xOffset,0);
    
    EVERY_N_MILLISECONDS(5000) {
      audioManager.printStatistics();
    }
  }
  
  const char *description() {
    return "SpectrumAnalyzer";
  }
};

/* ------------------------------------------------------------------------------------------------------ */

class MotionBlobs : public Pattern, public PaletteRotation<CRGBPalette256>  {
  static const bool kTestMode = false;
public:
  typedef enum { rect22, arrow, spikes } Shape;
private:
  /*--MotionBlob--*/
  class MotionBlob {
  public:
    MotionBlobs *paletteRotation;
    
    int y;
    int x;
    Shape shape;

    void reset(MotionBlobs *paletteRotation, int x, int y, Shape shape) {
      this->paletteRotation = paletteRotation;
      this->x = x;
      this->y = y;
      this->shape = shape;
    }
    
    void update(DrawingContext ctx, float theta, float dtheta) {
      ctx.pushStyle();
      ctx.drawStyle.boundsBehavior = DrawStyle::wrap;

      switch (shape) {
        case rect22: {
          float px = x + theta * TOTAL_WIDTH / 360;
          CRGB color = paletteRotation->getPaletteColor(beatsin8(3));
          ctx.point(px,  y,   color);
          ctx.point(px+1, y,   color);
          ctx.point(px,  y+1, color);
          ctx.point(px+1, y+1, color);
          break;
        }
        case arrow: {
          float px = x + theta * TOTAL_WIDTH / 360;
          float bend = 1.5*dtheta;

          for (int y = 0; y < ctx.height; ++y) {
            if (y < ctx.height/2) {
              px += 1 * bend;
            } else if (y > ctx.height/2) {
              px -= 1 * bend;
            }
            float arrowstart = px - 2 * bend;

            float yCenterDistanceFactor = (fabsf(ctx.height/2 - 0.5 - y) - 0.5) / (ctx.height/2 - 1);
            
            uint8_t paletteIndexTimeShift = beatsin16(1, 0, 2*0xFF);
            uint8_t paletteIndexCircularShift = 0xFF * x / (float)TOTAL_WIDTH;
            uint8_t paletteIndex = triwave8((uint8_t)(0x7F * yCenterDistanceFactor) + paletteIndexTimeShift + paletteIndexCircularShift);
            CRGB color = paletteRotation->getPaletteColor(paletteIndex);
            CRGB c1 = color;
            CRGB c2 = color;

            //  draw a second faded arrow
            float iptr;
            float frac = modff(arrowstart, &iptr);
            c1 = c1.nscale8_video(max(1, (1 - frac) * 0xFF));
            c2 = c2.nscale8_video(max(1, (frac * 0xFF)));
            
            // dim slightly at the top and bottom
            float dimmingFactor = 0.5;
            float heightDimmingAlpha = dimmingFactor * yCenterDistanceFactor;

            c1 = c1.nscale8_video(0xFF - (uint8_t)min(0xFF, 0xFF * heightDimmingAlpha));
            c2 = c2.nscale8_video(0xFF - (uint8_t)min(0xFF, 0xFF * heightDimmingAlpha));

            ctx.point((int)iptr, (int)y, c1, blendBrighten); 
            ctx.point((int)ceilf(arrowstart), (int)y, c2, blendBrighten);
          }
          break;
        }
        case spikes: {
          // vertical bar while still, morphs in 2D to a horizontal bar, passing through larger shapes matched to rotation velocity
          // while horizontal, forms a complete 2px thick line over entire belt that tracks 2 cycles of a palettes
          
          // 4 line segments, drawing a diamond of w,h
          // 2 line segments, drawing a cross of w,h
          // tune weights for how these parameters are matched to rotation velocity
          // track vertical motion too?

          float px = x + -2*theta * TOTAL_WIDTH / 360;
          float bend = min(ctx.height/2-1, max(0.01, abs(2*-dtheta)));          
          
          CRGB color1, color2;
          if (paletteRotation->colorMode == palette) {
            color1 = paletteRotation->getPaletteColor(triwave8(0xFF * (x+bend+1)/PANEL_WIDTH)); // FIXME: needs to used mirrored palette
            color2 = paletteRotation->getPaletteColor(triwave8(0xFF * (x-bend-1)/PANEL_WIDTH));
          } else {
            color1 = CHSV(0xFF * (x+bend+1)/PANEL_WIDTH, 0xFF, 0xFF);
            color2 = CHSV(0xFF * (x-bend-1)/PANEL_WIDTH, 0xFF, 0xFF);
          }

          // diamond
          ctx.line(px, bend, px+bend, ctx.height/2-1, color1, color2);
          ctx.line(px, bend, px-bend, (int)ctx.height/2-1, color1, color2);
          ctx.line(px, ctx.height-(int)bend-1, px+bend, ctx.height/2, color1, color2);
          ctx.line(px, ctx.height-(int)bend-1, px-bend, ctx.height/2, color1, color2);

          // cross
          // FIXME: better fadeups e.g. proper antialiasing
          ctx.line(px, bend, px, ctx.height-floorf(bend) - 1, color1.lerp8(color2, 0x7f), true);
          ctx.line(px-bend-1, ctx.height/2 - 1, px+bend+1, ctx.height/2 - 1, color1, color2);
          ctx.line(px-bend-1, ctx.height/2, px+bend+1, ctx.height/2, color1, color2);
          break;
        }
      }
      ctx.popStyle();
    }
  };
  /*--End of MotionBlob--*/
  
public:
  const int kBlobCount = 8;
  const int kBlobsResetTimeout = 5000;
  MotionBlob *blobs = NULL;
  long lastDisplay = -kBlobsResetTimeout;
  
  typedef enum { palette, rainbow } ColorMode;
  ColorMode colorMode;

  Shape mode;

  MotionBlobs(Shape mode) : PaletteRotation(minBrightness=10) {
    this->mode = mode;
    blobs = new MotionBlob[kBlobCount];
    colorMode = random8(2) ? palette : rainbow;
    motionManager.subscribe();
    this->minBrightness = 5;
  }

  ~MotionBlobs() {
    motionManager.unsubscribe();
    delete [] blobs;
  }

  void resetBlobs() {
    // space out the blobs so they don't overlap
    const int ybuckets = 2;
    const int xbuckets = kBlobCount;

    const int xbucketSize = ctx.width / xbuckets;
    // const int ybucketSize = ctx.height / ybuckets;
    
    for (int i = 0; i < kBlobCount; ++i) {
      int xbucketStart = (i * xbucketSize) % ctx.width;
      // int ybucketStart = ybucketSize * ((i * xbucketSize) / ctx.width);

      int x = xbucketStart;// + random8(xbucketSize);
      int y = 0;//ybucketStart + random8(ybucketSize - 1);
      
      blobs[i].reset(this, x, y, mode);
    }
  }

  void update() {
    ctx.leds.fadeToBlackBy(kTestMode ? 60 : 60);

    if (lastDisplay != -1 && millis() - lastDisplay > 5000) {
      resetBlobs();
      lastDisplay = -1;
    }

    float orientation;
    float twirlVelocity = motionManager.twirlVelocity(10, &orientation);
    if (kTestMode) {
      twirlVelocity = beatsin8(20, 0, 0xFF)/63. - 2;
      orientation = beatsin8(20, 0, 0xFF, 0, 195) - 127;
    }
    ctx.pushStyle();
    ctx.blendMode(blendBrighten);
    for (int i = 0; i < kBlobCount; ++i) {
      MotionBlob *blob = &blobs[i];
      blob->update(ctx, orientation, twirlVelocity);
    }
    ctx.popStyle();
    lastDisplay = millis();
  }
  
  bool wantsToIdleStop() {
    return true;
  }
};

class ArrowSpin : public MotionBlobs {
public:
  ArrowSpin() : MotionBlobs(MotionBlobs::arrow) {};
  const char *description() {
    return "ArrowSpin";
  }
};

class SpikeSpin : public MotionBlobs {
public:
  SpikeSpin() : MotionBlobs(MotionBlobs::spikes) {};
  const char *description() {
    return "SpikeSpin";
  }
};

/* ------------------------------------------------------------------------------------------------------ */

const uint8_t fontHeight = 7;
const char *fontN = ""
"10001"
"11001"
"10101"
"10011"
"10001"
"10001"
"10001";

const char *fontS = ""
"01110"
"10001"
"10000"
"01110"
"00001"
"10001"
"01110";

const char *fontE = ""
"11111"
"10000"
"10000"
"11110"
"10000"
"10000"
"11111";

const char *fontW = ""
"1000001"
"1000001"
"1001001"
"1001001"
"1001001"
"1010101"
"0100010";

class Glyph {
  CRGB *data=NULL;
  uint8_t width=0;
  uint8_t height=0;
public:
  Glyph(uint8_t width, uint8_t height, const char *datastr, std::map<char, CRGB> colormap) : width(width), height(height) {
    data = new CRGB[width * height];
    for (int i = 0; i < width * height; ++i) {
      data[width * height - i - 1] = colormap[datastr[i]];
    }
  }
  ~Glyph() {
    delete [] data;
  }
  uint8_t getWidth() { return width; }
  uint8_t getHeight() { return height; }
  
  template<unsigned WIDTH, unsigned HEIGHT, class PixelType, class PixelSetType>
  void draw(CustomDrawingContext<WIDTH, HEIGHT, PixelType, PixelSetType> &ctx, int drawx, int drawy, BlendMode blendMode=blendSourceOver) {
    if (data == NULL) return;
    ctx.pushStyle();
    ctx.drawStyle.boundsBehavior = DrawStyle::wrap;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        ctx.point(x+drawx, y+drawy, data[y*width + x], blendMode);
      }
    }
    ctx.popStyle();
  }
};

class Compass : public Pattern, public PaletteRotation<CRGBPalette32> {
  Glyph *cardinals[4];

  static const int frontGap = 4;
  static const int panelLength = 32; // yay about 1cm per pixel
  static const int backGap = 26;
  static const int circumference = frontGap + 2*panelLength + backGap;
  const int panelStartX[2] = {2, frontGap/2 + panelLength + backGap}; // measuring from front center
  
  // we'll create a buffer the full size of the belt to draw 360º compass to, and then copy the parts that "overlap" onto the panels
  CustomDrawingContext< circumference, TOTAL_HEIGHT, CRGB, CRGBArray<circumference * TOTAL_HEIGHT> > circleBuffer;

public:
  Compass() : PaletteRotation(minBrightness=10) {
    std::map<char,CRGB> colormap;
    colormap['0'] = CRGB::Black;
    colormap['1'] = CRGB::White;

    const char *letters[4] = {fontS, fontW, fontN, fontE};
    for (unsigned i = 0; i < ARRAY_SIZE(letters); ++i) {
      cardinals[i] = new Glyph(strlen(letters[i]) / fontHeight, fontHeight, letters[i], colormap);
    }
    motionManager.subscribe();
  }
  ~Compass() {
    motionManager.unsubscribe();
  }

  void update() {
    ctx.leds.fill_solid(CRGB::Black);
    circleBuffer.leds.fill_solid(CRGB::Black);

    sensors_event_t event;
    motionManager.getEvent(&event);
    float xOffset = (event.magnetic.x + 90) * circleBuffer.width / 360;
    // motionManager.printStatus();

    uint8_t systemCalibration = 0;
    motionManager.bno.getCalibration(&systemCalibration, NULL, NULL, NULL);
    if (systemCalibration == 0) {
      int dotSpacing = 8;
      for (int i = 0; i < ctx.width / dotSpacing; ++i) {
        ctx.point(mod_wrap(i*dotSpacing + event.orientation.x, ctx.width), 3, CRGB::White);
      }
    } else {
      for (unsigned i = 0; i < ARRAY_SIZE(cardinals); ++i) {
        int x = i * circleBuffer.width / ARRAY_SIZE(cardinals);
        int y = 0;
        cardinals[i]->draw(circleBuffer, x - cardinals[i]->getWidth()/2.0, y);
      }
      
      circleBuffer.shift_buffer(-xOffset,0);

      // now copy the larger buffer onto the panels
      for (int p = 0; p < PANEL_COUNT; ++p) {
        for (int y = 0; y < PANEL_HEIGHT; ++y) {
          for (int x = 0; x < PANEL_WIDTH; ++x) {
            ctx.ledat(x + p * PANEL_WIDTH,y) = circleBuffer.ledat(x + panelStartX[p], y);
          }
        }
      }
    }
  }

  const char *description() {
    return "Compass";
  }
};

/* ------------------------------------------------------------------------------------------------------ */

class Bars : public Pattern, public PaletteRotation<CRGBPalette16> {
  const int kSecondsPerPalette = 10;
  const int kBarWidth = 8;
  unsigned numBars = 0;
  
  float *xvelocities = NULL;
  float *xoffsets = NULL;
public:  
  Bars() {
    motionManager.subscribe();
  }
  
  ~Bars() {
    delete [] xvelocities;
    delete [] xoffsets;

    motionManager.unsubscribe();
  }

  void setup() {
    numBars = ctx.width / kBarWidth / 2 * ctx.height;

    prepareTrackedColors(numBars);
    
    xvelocities = new float[ctx.height];
    xoffsets = new float[ctx.height];

    for (int i = 0 ; i < ctx.height; ++i) {
      xvelocities[i] = random8() / (255.0 * 2) + 0.75;
      xoffsets[i] = random8(8);
    }
  }
  
  void update() {
    ctx.leds.fill_solid(CRGB::Black);

    EVERY_N_MILLISECONDS(40) {
      shiftTrackedColors(1);
    }
    
    const int rotationSamples = 20;
    float rotationVelocity = motionManager.twirlVelocity(rotationSamples);

    ctx.pushStyle();
    ctx.drawStyle.boundsBehavior = DrawStyle::wrap;
    
    for (int y = 0; y < ctx.height; ++y) {
      for (int x = 0; x < ctx.width / kBarWidth; ++x) {
        if (addmod8(x, y, 2) == 0) {
          continue;
        }
        float row_startx = (y & 1 ? xoffsets[y] : -xoffsets[y]);
        
        float x1 = row_startx + x*kBarWidth;
        float x2 = row_startx + (x+1)*kBarWidth - 1;

        CRGB color = getTrackedColor(y * ctx.width / kBarWidth / 2 + x / 2);
        ctx.line(x1, y, x2, y, color, true);
      }
      xoffsets[y] += xvelocities[y] * frameTime() / 30.0 * rotationVelocity;
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

class Oscillators : public Pattern, public PaletteRotation<CRGBPalette16>, public FFTProcessing {
  static const int fftBins = 5;
  float offset = 0;
  uint8_t mode;
  float fftHistory[fftBins][PANEL_HEIGHT];
public:
  Oscillators() : PaletteRotation(minBrightness=10), FFTProcessing(&audioManager, fftBins) {
    secondsPerPalette = 20;
    mode = 0;
    motionManager.subscribe();
    motionManager.addActivityHandler(JumpActivity, "oscillators", [this]() {
      this->mode = addmod8(this->mode, 1, 3);
    });
  }

  ~Oscillators() {
    motionManager.removeActivityHandler(JumpActivity, "oscillators");
    motionManager.unsubscribe();
  }

  void update() {
//    for (int index = 0; index < ctx.width * ctx.height; ++index) {
//      // this looks cool for a good long time
//      int r = 0;//beatsin16(millis() / 15 + 5 * index) >> 8;
//      int g = beatsin16(millis() / 10 + 10 * index) >> 8;
//      int b = 0;//beatsin16(millis() / 5 + 15 * index) >> 8;
//      EVERY_N_MILLISECONDS(10) {
//        logf("b1 = %i, b2 = %i", b1, b2);
//      }
//CRGB color = CRGB(dim8_raw(r), dim8_raw(g), dim8_raw(b));
//      ctx.leds[index] = color;
//    }

    // float offset = millis() / 2.;
    offset += motionManager.twirlVelocity(20);
    const float *levels = fftLevels(5);

    EVERY_N_MILLIS(32) {
      for (int i = 0; i < fftBins; ++i) {
        for (int y = PANEL_HEIGHT - 1; y > 0; --y) {
          fftHistory[i][y] = fftHistory[i][y-1];
        }
      }
    }
    for (int i = 0; i < fftBins; ++i) {
      fftHistory[i][0] = levels[i];
    }
    for (int x = 0; x < ctx.width; ++x) {
      for (int y = 0; y < ctx.height; ++y) {
        int phase = 10 * offset + x*39 + 4*offset*y;
        float bucket = fmod_wrap((phase+64) / 255., fftBins);
        uint8_t colorIndex = 255./fftBins * (int)bucket;
        CRGB color;
        switch (mode) {
          case 1:
            color = CHSV(colorIndex, 0xFF, 0xFF); break;
          case 2:
            color = ColorFromPalette((CRGBPalette32)Trans_Flag_gp, colorIndex + 0xFF/5); break;
          case 0:
          default:
           color = getPaletteColor(addmod8(colorIndex, 51, 0xFF)); break;
        }
        
        color.nscale8_video(min(0xFF, scale8(dim8_video(sin8(phase)), fftHistory[(int)bucket % fftBins][y] * 1 * 0xFF)));
        if (color.r + color.g + color.b < 3) {
          color = CRGB::Black;
        }
        ctx.point(x, y, color);
      }
    }
    offset *= 0.98;
  }
  const char *description() {
    return "Oscillators";
  }
};

/* ------------------------------------------------------------------------------------------------------ */

class RainbowMotion : public Pattern {
  ~RainbowMotion() {
    motionManager.unsubscribe();
  }

  bool wantsToIdleStop() {
    return true;
  }
  
  void setup() {
    motionManager.subscribe();
  }

  void update() {
    CRGBArray<NUM_LEDS> leds = ctx.leds;
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

class PixelDust : public Pattern, public PaletteRotation<CRGBPalette16> {
private:
  static const unsigned int numParticles = 50;
  Adafruit_PixelDust *pixelDust[PANEL_COUNT];
public:
  PixelDust() : PaletteRotation(minBrightness=3) {
    for (int i = 0; i < PANEL_COUNT; ++i) {
      pixelDust[i] = new Adafruit_PixelDust(PANEL_WIDTH, PANEL_HEIGHT, numParticles, 0xFF, 101, false);
    }

    for (int i = 0; i < PANEL_COUNT; ++i) {
      if(!pixelDust[i]->begin()) {
        logf("PixelDust init failed");
      }
      pixelDust[i]->randomize();
    }

    prepareTrackedColors(numParticles);
    motionManager.subscribe();
  }

  ~PixelDust() {
    motionManager.unsubscribe();
  }

  void update() {
    CRGBArray<NUM_LEDS> leds = ctx.leds;
    dimension_t x, y;
    sensors_event_t accel;
    sensors_event_t linear_accel;
    motionManager.bno.getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    motionManager.bno.getEvent(&linear_accel, Adafruit_BNO055::VECTOR_LINEARACCEL);
    float jerkFactor = 2.0;

    // Z through board (right-side-up positive)
    // X vertical with text
    // Y horizontal along board

    // apply a small correction factor to Z to account for the board not sitting perfectly vertical on the belt/my back, pixels tend to get stuck towards the back

    float zcorrect[2] = {0};

    for (int p = 0; p < 2; ++p) {
      int zbalance = 0;
      for (unsigned n = 0; n < numParticles; ++n) {
        dimension_t x,y;
        pixelDust[0]->getPosition(n, &x, &y);
        if (x > PANEL_WIDTH/2) {
          zbalance++;
        } else {
          zbalance--;
        }
      }
      zcorrect[p] = 2.0 * zbalance / numParticles;
      // logf("zbalance %s = %i, zcorrect = %f", p==0?"right":"left", zbalance, zcorrect[p]);
    }

    // logf("ACCEL (%0.2f, %0.2f, %0.2f), LINACCEL (%0.2f, %0.2f, %0.2f)", accel.acceleration.x,accel.acceleration.y,accel.acceleration.z,
    //                                                                     linear_accel.acceleration.x, linear_accel.acceleration.y, linear_accel.acceleration.z);

    // board x impacted by motion z (through board) and motion y (horizontal along board). 
    //   disinclude accel.y in favor of linear_accel.y  because we want to capture shifting and twirling, but wouldn't know what to do with wrapped panels if gravity is to the side
    // board y impacted by motion x (vertical along board). add jerk factor in order to make pixels bounce

    pixelDust[0]->iterate(
                  -1 * (accel.acceleration.z + jerkFactor * linear_accel.acceleration.z + zcorrect[0] - jerkFactor * linear_accel.acceleration.y),
                  1 * (accel.acceleration.x + jerkFactor * linear_accel.acceleration.x), 
                  0);
    
    pixelDust[1]->iterate(
                  1 * (accel.acceleration.z + jerkFactor * linear_accel.acceleration.z + zcorrect[1] - jerkFactor * linear_accel.acceleration.y),
                  1 * (accel.acceleration.x + jerkFactor * linear_accel.acceleration.x), 
                  0);

    leds.fill_solid(CRGB::Black);

    for (int i = 0; i < PANEL_COUNT; ++i) {
      for (unsigned int p = 0; p < numParticles; p++) {
        pixelDust[i]->getPosition(p, &x, &y);
        leds[ledxy(x + i * PANEL_WIDTH, y)] = getTrackedColor(p);
      }
    }
  }

  const char *description() {
    return "PixelDust";
  }
};


/* ------------------------------------------------------------------------------------------------------ */

class Droplets : public Pattern, public PaletteRotation<CRGBPalette256>, public FFTProcessing {
private:
  unsigned long lastDrop;
  unsigned long lastFlow;
  FCRGB scratch[NUM_LEDS];
  CustomDrawingContext< TOTAL_WIDTH, TOTAL_HEIGHT, FCRGB, FCRGBArray<NUM_LEDS> > accumCtx;
  
  void flowDroplets(int i, int i2) {

    // TODO: Look into replacing this with blur2d

    const float kFlow = 0.1;
    const float kEff = 0.70;
    const float minLoss = 1.5;

    FCRGB led1 = accumCtx.leds[i];
    FCRGB led2 = accumCtx.leds[i2];
    for (uint8_t sp = 0; sp < 3; ++sp) { // each subpixel
      float *refSp = NULL;
      float *srcSp = NULL;
      float *dstSp = NULL;
      if (led1[sp] < led2[sp]) {
        refSp = &led2[sp];
        srcSp = &scratch[i2][sp];
        dstSp = &scratch[i][sp];
      } else if (led1[sp] > led2[sp] ) {
        refSp = &led1[sp];
        srcSp = &scratch[i][sp];
        dstSp = &scratch[i2][sp];
      }
      if (srcSp && dstSp) {
        float flow = min(*srcSp, min((kFlow * *refSp), 0xFF - *dstSp));
        *dstSp += kEff * flow;
        if (*dstSp < 0.001) *dstSp = 0;
        *srcSp -= min(*srcSp, max(minLoss, flow));
      }
    }
  }
  static const int fftBinCount = 10;
  const float *levels = NULL;
public:
  Droplets() : PaletteRotation(minBrightness=15), FFTProcessing(&audioManager, fftBinCount) {
    secondsPerPalette = 15;
    motionManager.subscribe();
  }
  ~Droplets() {
    motionManager.unsubscribe();
  }

  void setup() {
    // copy the whole standard pixel buffer into a floating-point pixel buffer
    for (int i = 0; i < NUM_LEDS; ++i) {
      accumCtx.leds[i].red = ctx.leds[i].r;
      accumCtx.leds[i].green = ctx.leds[i].g;
      accumCtx.leds[i].blue = ctx.leds[i].b;
    }
  }

  float extraGain = 1.0;
  void update() {
    CRGBArray<NUM_LEDS> leds = ctx.leds;
    const unsigned int flowInterval = 30;

    paletteRotationTick();
    
    levels = NULL;
    if (fftAvailable()) {
      levels = fftLevels(2);
    }
    
    if (levels != NULL) {
      for (int i = 0; i < fftBinCount; ++i) {
        float level = levels[i] * extraGain;
        if (level > 0.03) {
          for (int p = 0; p < PANEL_COUNT; ++p) {
            float dropx = 2*p * PANEL_WIDTH + (p?-1:1) * PANEL_WIDTH * i / (float)fftBinCount + (cos8(random8()) - 127) / 32;
            float dropy = random16(ctx.height * 4) / 4.0;
            CRGB color = getPaletteColor(i/(float)fftBinCount * 0x100);
            color.nscale8_video(max(0, min(0xFF, 0xFF * level / 0.15)));
            accumCtx.circle(dropx, dropy, min(2, min(4, level * 12)), 8, true, FCRGB(color));
            fftSetLevelAccum(i, level / 2);
          }
        }
      }
    }

    unsigned long mils = millis();

    // for non-sound-based droplets
    // const unsigned int dropInterval = 80;
    // if (mils - lastDrop > dropInterval) {
    //   float dropx = random16(ctx.width * 4) / 4.0;
    //   float dropy = random16(ctx.height * 4) / 4.0;
    //   CRGB color = getPaletteColor(random8());
    //   accumCtx->circle(dropx, dropy, 2, 8, true, FCRGB(color));
    //   lastDrop = mils;
    // }

    if (mils - lastFlow > flowInterval) {
      for (int i = 0; i < NUM_LEDS; ++i) {
        scratch[i] = accumCtx.leds[i];
      }
      for (int p = 0; p < PANEL_COUNT; ++p) {
        for (int x = 0; x < PANEL_WIDTH; ++x) {
          for (int y = 0; y < PANEL_HEIGHT; ++y) {
            int i = ledxy(p * PANEL_WIDTH + x, y);
            if (x < PANEL_WIDTH - 1) {
              int i2 = ledxy(p * PANEL_WIDTH + x+1, y);
              // calculate flows from og leds, set in scratch
              flowDroplets(i, i2);
            }
            if (y < PANEL_HEIGHT - 1) {
              int i3 = ledxy(p * PANEL_WIDTH + x, y+1);
              // calculate flows from og leds, set in scratch
              flowDroplets(i, i3);
            }
          }
        }
      }
      for (int i = 0; i < NUM_LEDS; ++i) {
        accumCtx.leds[i] = scratch[i];
      }
      lastFlow  = mils;
    }
    int ledsLit = 0;
    for (int i = 0; i < NUM_LEDS; ++i) {
        leds[i].red = ceilf(accumCtx.leds[i].red);
        leds[i].green = ceilf(accumCtx.leds[i].green);
        leds[i].blue = ceilf(accumCtx.leds[i].blue);

        if (leds[i]) {
          ledsLit++;
        }
    }
    float litRatio = ledsLit / (float)NUM_LEDS;
    const float litUpperThresh = 0.85;
    if (litRatio > litUpperThresh) {
      extraGain = max(0, extraGain - (litRatio - litUpperThresh) / 80);
    }
    if (ledsLit < 0.15 * NUM_LEDS && extraGain < 2) {
      extraGain += 0.001 / (extraGain > 1 ? extraGain * extraGain : 1.0);
    }

    sensors_event_t event;
    motionManager.getEvent(&event);
    float xOffset = event.orientation.x * TOTAL_WIDTH / 360;
    ctx.shift_buffer(-xOffset,0);
  }

  const char *description() {
    return "Droplets";
  }
};

/* ------------------------------------------------------------------------------------------------------ */

class Triangles : public Pattern, public PaletteRotation<CRGBPalette16> {//, public FFTProcessing {
public:
  Triangles() {// : FFTProcessing(&audioManager, 10) {
    ctx.drawStyle.boundsBehavior = DrawStyle::wrapX;
  }
  ~Triangles() {
    motionManager.unsubscribe();
  }

  void setup() {
    motionManager.subscribe();
  }

  float theta = 0;
  void update() {
    ctx.leds.fill_solid(CRGB::Black);
    const float midY = TOTAL_HEIGHT/2-0.5;

    // ctx.wuline(0,1,16,6,CRGB::White, CRGB::Red);
    // return;
    int count = 5;
    float r = beatsin8(10, 40, 120) / 20.;
    float xbase = beatsin8(6, 0, 64) / 4.;
    for (int i = 0 ; i < count; ++i) {
      float x1 = xbase + i * TOTAL_WIDTH/count + r*sinf(theta + 0);
      float x2 = xbase+ i * TOTAL_WIDTH/count + r*sinf(theta + 2*M_PI/3);
      float x3 = xbase + i * TOTAL_WIDTH/count + r*sinf(theta + 4*M_PI/3);
      float y1 = midY + r*cosf(theta + 0);
      float y2 = midY + r*cosf(theta + 2*M_PI/3);
      float y3 = midY + r*cosf(theta + 4*M_PI/3);
      ctx.line(x1,y1,x2,y2,CRGB::Red, CRGB::Green);
      ctx.line(x2,y2,x3,y3,CRGB::Green, CRGB::Blue);
      ctx.line(x3,y3,x1,y1,CRGB::Blue, CRGB::Red);
    }
    theta += 0.05;
  }

  const char *description() {
    return "Triangles";
  }
};


/* ------------------------------------------------------------------------------------------------------ */

#define SECONDS_PER_PALETTE 20
uint8_t gCurrentPaletteNumber = 0;
CRGBPalette16 gCurrentPalette( CRGB::Black);
CRGBPalette16 gTargetPalette( gGradientPalettes[0] );

class SmoothPalettes : public Pattern {
    void setup() {
      gTargetPalette = gGradientPalettes[random16(gGradientPaletteCount)];
    }
    void update() {
      CRGBArray<NUM_LEDS> leds = ctx.leds;
      EVERY_N_MILLISECONDS(20) {
        draw(leds);
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
