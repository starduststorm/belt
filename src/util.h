#ifndef UTIL_H
#define UTIL_H

#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))

#if DEBUG
#define assert(expr, reason) if (!(expr)) { logf("ASSERTION FAILED: %s", reason); while (1) delay(100); }
#else
#define assert(expr, reason) if (!(expr)) { logf("ASSERTION FAILED: %s", reason); }
#endif

static int vasprintf(char** strp, const char* fmt, va_list ap) {
  va_list ap2;
  va_copy(ap2, ap);
  char tmp[1];
  int size = vsnprintf(tmp, 1, fmt, ap2);
  if (size <= 0) {
    strp=NULL;
    return size;
  }
  va_end(ap2);
  size += 1;
  *strp = (char*)malloc(size * sizeof(char));
  return vsnprintf(*strp, size, fmt, ap);
}

void logf(const char *format, ...)
{
  va_list argptr;
  va_start(argptr, format);
  char *buf;
  vasprintf(&buf, format, argptr);
  va_end(argptr);
  Serial.println(buf ? buf : "LOGF MEMORY ERROR");
#if DEBUG
  Serial.flush();
#endif
  free(buf);
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
    logf("ledxy: OUT OF BOUNDS AT %i,%i", x,y);
    while(1) delay(100);
  } else if (y < 0 || y >= PANEL_HEIGHT) {
    logf("ledxy: OUT OF BOUNDS AT %i,%i", x,y);
    while(1) delay(100);
  }
#endif
  unsigned int index = 0;
  if (x & 1) {
    index = (x + 1) * PANEL_HEIGHT - 1 - y;
  } else {
    index= x * PANEL_HEIGHT + y;
  }
  return index;
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

// lerp
float remap(float x, float oldmin, float oldmax, float newmin, float newmax) {
  float zero_to_one = (x-oldmin) / (oldmax-oldmin);
  return zero_to_one*(newmax-newmin) + newmin;
}

#endif