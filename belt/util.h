#ifndef UTIL_H
#define UTIL_H

#define MIC_PIN 14

#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))

#define assert(expr, reason) if (!(expr)) { logf("Assertion failed: %s", reason); }

void logf(const char *format, ...)
{
#if SERIAL_LOGGING
  va_list argptr;
  va_start(argptr, format);
  char *buf = (char *)calloc(strlen(format) + 200, sizeof(char));
  vsnprintf(buf, 200, format, argptr);
  va_end(argptr);
  Serial.println(buf);
#if DEBUG
  Serial.flush();
#endif
  free(buf);
#endif
}

#define MOD_DISTANCE(a, b, m) (abs(m / 2. - fmod((3 * m) / 2 + a - b, m)))

inline int mod_wrap(int x, int m) {
  int result = x % m;
  return result < 0 ? result + m : result;
}

inline float fmod_wrap(float x, int m) {
  float result = fmod(x, m);
  return result < 0 ? result + m : result;
}


inline unsigned int ledxy(int x, int y) {
  // support zigzag
#if DEBUG
  if (x < 0 || x >= PANEL_WIDTH * PANEL_COUNT) {
    logf("ledrc: OUT OF BOUNDS AT %i,%i", x,y);
    while(1);
  } else if (y < 0 || y >= PANEL_HEIGHT) {
    logf("ledrc: OUT OF BOUNDS AT %i,%i", x,y);
    while(1);
  }
#endif
  unsigned int index = 0;
  if (x & 1) {
    index = (x + 1) * STRIP_LENGTH - 1 - y;
  } else {
    index= x * STRIP_LENGTH + y;
  }
  return index;
}

inline unsigned int ledrc(int r, int c) {
  // FIXME: remove, name was awful
  return ledxy(r, c);
}

class FrameCounter {
  private:
    long lastPrint = 0;
    long frames = 0;
    long lastClamp = 0;
  public:
    long printInterval = 2000;
    void tick() {
      unsigned long mil = millis();
      long elapsed = mil - lastPrint;
      if (elapsed > printInterval) {
        if (lastPrint != 0) {
          logf("Framerate: %f", frames / (float)elapsed * 1000);
        }
        frames = 0;
        lastPrint = mil;
      }
      ++frames;
    }
    void clampToFramerate(int fps) {
      int delayms = 1000 / fps - (millis() - lastClamp);
      if (delayms > 0) {
        delay(delayms);
      }
      lastClamp = millis();
    }
};

// FIXME: replace all uses with cos8 with clamping from FastLED
float util_cos(float x, float offset, float period, float minn, float maxx) {
  float value = cos((x/period - offset) * M_PI * 2) / 2 + 0.5;
  return value*(maxx-minn) + minn;
}

// clamp in FastLED?
float clamp(float x, float minn, float maxx) {
  return fmax(minn, fmin(maxx, x));
}

// lerp
float remap(float x, float oldmin, float oldmax, float newmin, float newmax) {
  float zero_to_one = (x-oldmin) / (oldmax-oldmin);
  return zero_to_one*(newmax-newmin) + newmin;
}

#endif
