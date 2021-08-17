
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
  bool wrap = false;
};

template<class PixelType, class BufferType>
class CustomDrawingContext {
private:
  std::stack<DrawStyle> styleStack;
public:
  DrawStyle drawStyle;
  int width, height;
  BufferType &leds;
  CustomDrawingContext(BufferType &leds, unsigned width, unsigned height) : leds(leds) {
    this->width = width;
    this->height = height;
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
  
  void point(int x, int y, PixelType src, BlendMode blendMode) {
    if (drawStyle.wrap) {
      x = mod_wrap(x, width);
      y = mod_wrap(y, width);
    } else {
      if (x < 0 || x >= width || y < 0 || y >= height) {
        logf("point: (%i, %i) out of bounds", x, y);
        // FIXME: is this more useful than asserting? or just assert during debug?
        return;
      }
      assert(x >=0 && x < width, "point: x out of bounds");
      assert(y >=0 && y < height, "point: y out of bounds");
    }
    int index = ledxy(x,y);
    switch (blendMode) {
      case blendSourceOver: leds[index] = src; break;
      case blendBrighten: {
        PixelType dst = leds[index];
        leds[index] = PixelType(max(src.r, dst.r), max(src.g, dst.g), max(src.b, dst.b));
        break;
      }
      case blendDarken: {
        PixelType dst = leds[index];
        leds[index] = PixelType(min(src.r, dst.r), min(src.g, dst.g), min(src.b, dst.b));
      }
      case blendAdd: {
        PixelType dst = leds[index];
        leds[index] = PixelType(min(0xFF, (float)src.r + (float)dst.r), min(0xFF, (float)src.g + (float)dst.g), min(0xFF, (uint16_t)src.b + (uint16_t)dst.b));
      }
    }
  }

  void point(int x, int y, PixelType src) {
    point(x, y, src, drawStyle.blendMode);
  }
  
  void line(float x1, float y1, float x2, float y2, PixelType src, bool antialias=false) {
    // TODO: fix antialiasing for diagonal lines
    bool useY = (x1 == x2 || fabsf((y2 - y1) / (float)(x2 - x1)) > 1);
    if (useY) {
      if (y1 == y2) {
        // this is just a point
        point(x1, y1, src);
        return;
      }
      if (y1 > y2) {
        std::swap(y1, y2);
        std::swap(x1, x2);
      }
      for (float y = y1; y <= y2; ++y) {
        // TODO: can I do this without the float divisions?
        float x = x1 + (y - y1) / (float)(y2 - y1) * (x2 - x1);
        if (antialias) {
          float iptr;
          float frac = fabsf(modff(y, &iptr));
          float partial1 = floorf(y);
          float partial2 = ceilf(y);
          if (y < 0) {
            std::swap(partial1, partial2);
          }
          PixelType src1 = src;
          PixelType src2 = src;
          point(round(x), partial1, src1.nscale8_video(0xFF * (1-frac)), blendAdd);
          if (frac > 0) {
            point(round(x), partial2, src2.nscale8_video(0xFF * frac), blendAdd);
          }
        } else {
          point(round(x), round(y), src);
        }
      }
    } else {
      if (x1 > x2) {
        std::swap(y1, y2);
        std::swap(x1, x2);
      }
      for (float x = x1; x <= x2 + 0.0001; ++x) {
        float y = y1 + (x - x1) / (float)(x2 - x1) * (y2 - y1);
        if (antialias) {
          float iptr;
          float frac = fabsf(modff(x, &iptr));
          float partial1 = floorf(x);
          float partial2 = ceilf(x);
          if (x < 0) {
            std::swap(partial1, partial2);
          }
          PixelType src1 = src;
          PixelType src2 = src;
          point(partial1, round(y), src1.nscale8_video(0xFF * (1-frac)), blendAdd);
          if (frac > 0) {
            point(partial2, round(y), src2.nscale8_video(0xFF * frac), blendAdd);
          }
        } else {
          point(round(x), round(y), src);
        }
      }
    }
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

typedef CustomDrawingContext< CRGB, CRGBArray<NUM_LEDS> > DrawingContext;

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
} FCRGB;

template<int SIZE>
class FCRGBArray {
  FCRGB entries[SIZE];
public:
  inline FCRGB& operator[] (uint16_t x) __attribute__((always_inline)) {
    return entries[x];
  };
};
