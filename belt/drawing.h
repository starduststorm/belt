
#include <stack>

// Workaround for linker issues when using copy-constructors for DrawStyle struct (since something is built with -fno-exceptions)
// https://forum.pjrc.com/threads/57192-Teensy-4-0-linker-issues-with-STL-libraries
extern "C"{
  int __exidx_start(){ return -1;}
  int __exidx_end(){ return -1; }
}
//

enum BlendMode {
  sourceOver, brighten, darken, /* add blending? but how to encode alpha? as a DrawStyle? */
};

struct DrawStyle {
public:
  BlendMode blendMode = sourceOver;
  bool wrap = false;
};

class DrawingContext {
private:
  std::stack<DrawStyle> styleStack;
public:
  DrawStyle drawStyle;
  int width, height;
  CRGBArray<NUM_LEDS> &leds;
  DrawingContext(CRGBArray<NUM_LEDS> &leds, unsigned width, unsigned height) : leds(leds) {
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
  
  void point(int x, int y, CRGB src) {
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
    switch (drawStyle.blendMode) {
      case sourceOver: leds[index] = src; break;
      case brighten: {
        CRGB dst = leds[index];
        leds[index] = CRGB(max(src.r, dst.r), max(src.g, dst.g), max(src.b, dst.b));
        break;
      }
      case darken: {
        CRGB dst = leds[index];
        leds[index] = CRGB(min(src.r, dst.r), min(src.g, dst.g), min(src.b, dst.b));
      }
    }
  }
  
  void line(float x1, float y1, float x2, float y2, CRGB src) {
    // TODO: anti-alias when floats are passed. make this the integer version.
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
        point(round(x), round(y), src);
      }
    } else {
      if (x1 > x2) {
        std::swap(y1, y2);
        std::swap(x1, x2);
      }
      for (float x = x1; x <= x2; ++x) {
        float y = y1 + (x - x1) / (float)(x2 - x1) * (y2 - y1);
        point(round(x), round(y), src);
      }
    }
  }

  void circle(float centerX, float centerY, float radius, int ndiv, bool fill, CRGB src) {
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
