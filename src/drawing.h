
#include <stack>

// Workaround for linker issues when using copy-constructors for DrawStyle struct (since something is built with -fno-exceptions)
// https://forum.pjrc.com/threads/57192-Teensy-4-0-linker-issues-with-STL-libraries
extern "C"{
  int __exidx_start(){ return -1;}
  int __exidx_end(){ return -1; }
}
//

enum BlendMode {
  blendSourceOver, blendBrighten, blendDarken, blendAdd, /* add blending? but how to encode alpha? as a DrawStyle? */
};

struct DrawStyle {
public:
  BlendMode blendMode = blendSourceOver;
  enum BoundsBehavior { warn,wrap,clip,wrapX } boundsBehavior;
};

template<unsigned WIDTH, unsigned HEIGHT, class PixelType, class PixelSetType>
class CustomDrawingContext {
private:
  std::stack<DrawStyle> styleStack;

  void set_px(PixelType src, int index, BlendMode blendMode, uint8_t brightness) {
    src.nscale8(brightness);
    switch (blendMode) {
      case blendSourceOver:
        leds[index] = src;
        break;
      case blendBrighten: {
        PixelType dst = leds[index];
        leds[index] = PixelType(std::max(src.r, dst.r), std::max(src.g, dst.g), std::max(src.b, dst.b));
        break;
      }
      case blendDarken: {
        PixelType dst = leds[index];
        leds[index] = PixelType(std::min(src.r, dst.r), std::min(src.g, dst.g), std::min(src.b, dst.b));
      }
      case blendAdd: {
        PixelType dst = leds[index];
        // FIXME: float hard coded
        leds[index] = PixelType(min(0xFF, (float)src.r + (float)dst.r), min(0xFF, (float)src.g + (float)dst.g), min(0xFF, (float)src.b + (float)dst.b));
      }

    }
  }

public:
  DrawStyle drawStyle;
  int width, height;
  PixelSetType leds;
  CustomDrawingContext() {
    this->width = WIDTH;
    this->height = HEIGHT;
  }

  void shift_buffer(float xshift, float yshift) {
    xshift = fmod_wrap(xshift, width);
    yshift = fmod_wrap(yshift, height);
    PixelSetType oldleds;
    oldleds = leds;
    
    float xshift_whole;
    float xshift_frac = fabsf(modff(xshift, &xshift_whole));
    uint8_t xshift_frac8 = xshift_frac * 0xFF;

    float yshift_whole;
    float yshift_frac = fabsf(modff(yshift, &yshift_whole));
    uint8_t yshift_frac8 = yshift_frac * 0xFF;

    for (int x = 0; x < width; ++x) {
      for (int y = 0; y < height; ++y) {
        CRGB samples[4];
        samples[0] = oldleds[ledxy(x - (int)xshift_whole, y - (int)yshift_whole, width, height, true)];
        samples[1] = oldleds[ledxy(x - (int)xshift_whole - (int)ceilf(xshift_frac), y - (int)yshift_whole, width, height, true)];
        samples[2] = oldleds[ledxy(x - (int)xshift_whole, y - (int)yshift_whole - (int)ceilf(yshift_frac), width, height, true)];
        samples[3] = oldleds[ledxy(x - (int)xshift_whole - (int)ceilf(xshift_frac), y - (int)yshift_whole - (int)ceilf(yshift_frac), width, height, true)];

        CRGB ref1 = blend(samples[0], samples[1], xshift_frac8);
        CRGB ref2 = blend(samples[2], samples[3], xshift_frac8);
        CRGB newpx = blend(ref1, ref2, yshift_frac8);

        leds[ledxy(x,y, width, height)] = newpx;
      }
    }
  }

  void blendIntoContext(CustomDrawingContext<WIDTH, HEIGHT, PixelType, PixelSetType> &otherContext, BlendMode blendMode, uint8_t brightness=0xFF) {
    assert(otherContext.leds.size() == this->leds.size(), "context blending requires same-size buffers");
    for (int i = 0; i < leds.size(); ++i) {
      otherContext.set_px(leds[i], i, blendMode, brightness);
    }
  }
  
  void blendMode(BlendMode mode) {
    drawStyle.blendMode = mode;
  }
  
  void pushStyle() {
    styleStack.push(drawStyle);
    drawStyle = DrawStyle();
  }
  
  void popStyle() {
    assert(!styleStack.empty(), "No style to pop");
    drawStyle = styleStack.top();
    styleStack.pop();
  }

  CRGB &ledat(int x, int y) {
    return leds[ledxy(x,y,width,height)];
  }
  
  void point(int x, int y, PixelType src, BlendMode blendMode) {
    switch (drawStyle.boundsBehavior) {
      case DrawStyle::wrap:
        x = mod_wrap(x, width);
        y = mod_wrap(y, height);
        break;
      case DrawStyle::warn:
        if (x < 0 || x >= width || y < 0 || y >= height) {
          logf("point: (%i, %i) out of bounds", x, y);
          // FIXME: is this more useful than asserting? or just assert during debug?
        }
        assert(x >=0 && x < width, "point: x out of bounds");
        assert(y >=0 && y < height, "point: y out of bounds");
        break;
      case DrawStyle::clip:
        if (x < 0 || x >= width || y < 0 || y >= height) {
          return;
        }
        break;
      case DrawStyle::wrapX:
        if (y < 0 || y >= height) {
          return;
        }
        x = mod_wrap(x, width);
        break;
    }
    int index = ledxy(x, y, width, height);
    set_px(src, index, blendMode, 0xFF);
  }

  void point(int x, int y, PixelType src) {
    point(x, y, src, drawStyle.blendMode);
  }
  
  void point(int x, int y, PixelType src, float brightness) {
    src.nscale8_video(brightness * 0xFF);
    point(x, y, src);
  }

  // Wu's algorithm
  static float fpart(float x) {
    return x - floor(x);
  }
  static float rfpart(float x) {
    return 1 - fpart(x);
  }
  void line(float x0, float y0, float x1, float y1, PixelType src0, PixelType src1) {
    bool steep = fabs(y1 - y0) > fabs(x1 - x0);
    
    if (steep) {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
        std::swap(src0, src1);
    }
    
    float dx = x1 - x0;
    float dy = y1 - y0;

    float gradient = (dx == 0.0 ? 1.0 : dy/dx);

    // handle first endpoint
    float xend = roundf(x0);
    float yend = y0 + gradient * (xend - x0);
    float xgap = rfpart(x0 + 0.5);
    float xpxl1 = xend; // this will be used in the main loop
    float ypxl1 = floorf(yend);
    if (steep) {
      point(ypxl1,   xpxl1, src0, rfpart(yend) * xgap);
      point(ypxl1+1, xpxl1, src0,  fpart(yend) * xgap);
    } else {
      point(xpxl1, ypxl1  , src0, rfpart(yend) * xgap);
      point(xpxl1, ypxl1+1, src0,  fpart(yend) * xgap);
    }
    float intery = yend + gradient; // first y-intersection for the main loop
    
    // handle second endpoint
    xend = roundf(x1);
    yend = y1 + gradient * (xend - x1);
    xgap = fpart(x1 + 0.5);
    float xpxl2 = xend; //this will be used in the main loop
    float ypxl2 = floorf(yend);
    if (steep) {
      point(ypxl2  , xpxl2, src1, rfpart(yend) * xgap);
      point(ypxl2+1, xpxl2, src1,  fpart(yend) * xgap);
    } else {
      point(xpxl2, ypxl2,   src1, rfpart(yend) * xgap);
      point(xpxl2, ypxl2+1, src1,  fpart(yend) * xgap);
    }
    
    // main loop
    if (steep) {
      assert(xpxl1 <= xpxl2, "xpxl1 <= xpxl2");
      for (float x = xpxl1 + 1; x <= xpxl2 - 1; ++x) {
        PixelType c = src0.lerp8(src1, 0xFF * (uint8_t)(x-(xpxl1 + 1)) / (uint8_t)(xpxl2 - 1-(xpxl1 + 1)));
        point(floorf(intery)  , x, c, rfpart(intery));
        point(floorf(intery)+1, x, c, fpart(intery));
        intery = intery + gradient;
      }
    } else {
      assert(xpxl1 <= xpxl2, "xpxl1 <= xpxl2");
        for (float x = xpxl1 + 1; x <= xpxl2 - 1; ++x) {
          PixelType c = src0.lerp8(src1, 0xFF * (uint8_t)(x-(xpxl1 + 1)) / (uint8_t)(xpxl2 - 1-(xpxl1 + 1)));
          point(x, floorf(intery),   c, rfpart(intery));
          point(x, floorf(intery)+1, c,  fpart(intery));
          intery = intery + gradient;
        }
    }
  }

  void line(float x1, float y1, float x2, float y2, PixelType src) {
    line(x1, y1, x2, y2, src, src);
  }

  void circle(float centerX, float centerY, float radius, int ndiv, bool fill, PixelType src) {
    // TODO: use sin8 & cos8 for perf
    for (int t = 0; t < 360; t += 360/ndiv) {
      int x = round(cos(t*M_PI/180.) * radius + centerX);
      int y = round(sin(t*M_PI/180.) * radius + centerY);
      if (x >= 0 && x < width && y >= 0 && y < height) {
        point(x, y, src);
      }
    }
    if (fill) {
      // TODO: lol horribly inefficient. this should use lines.
      if (radius > 0.5) {
        circle(centerX, centerY, radius-0.5, max(ndiv-1,4), fill, src);
      }
    }
  }
};

typedef CustomDrawingContext< TOTAL_WIDTH, TOTAL_HEIGHT, CRGB, CRGBArray<NUM_LEDS> > DrawingContext;

/* Floating-point pixel buffer support */

typedef struct FCRGB {
  union {
    struct {
      union {
        float r;
        float red;
      };
      union {
        float g;
        float green;
      };
      union {
        float b;
        float blue;
      };
    };
    float raw[3];
  };
public:
  FCRGB() { }
  FCRGB(CRGB color) : red(color.r), green(color.g), blue(color.b) { }
  FCRGB(float r, float g, float b) : red(r), green(g), blue(b) { }
  inline float& operator[] (uint8_t x) __attribute__((always_inline)) {
    return raw[x];
  }
  void nscale8(uint8_t brightness) {
    float scale = (float)brightness / 0xFF;
    r *= scale;
    g *= scale;
    b *= scale;
  }
} FCRGB;

template<int SIZE>
class FCRGBArray {
  FCRGB entries[SIZE];
public:
  inline FCRGB& operator[] (uint16_t x) __attribute__((always_inline)) {
    return entries[x];
  };
};
