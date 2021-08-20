#ifndef UTIL_H
#define UTIL_H

#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))

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

static void _logf(bool newline, const char *format, va_list argptr)
{
  if (strlen(format) == 0) {
    if (newline) {
      Serial.println();
    }
    return;
  }
  char *buf;
  vasprintf(&buf, format, argptr);
  if (newline) {
    Serial.println(buf ? buf : "LOGF MEMORY ERROR");
  } else {
    Serial.print(buf ? buf : "LOGF MEMORY ERROR");
  }
#if DEBUG
  Serial.flush();
#endif
  free(buf);
}

void logf(const char *format, ...)
{
  va_list argptr;
  va_start(argptr, format);
  _logf(true, format, argptr);
  va_end(argptr);
}

void loglf(const char *format, ...)
{
  va_list argptr;
  va_start(argptr, format);
  _logf(false, format, argptr);
  va_end(argptr);
}

#define assert(expr, reasonFormat, ...) assert_func((expr), #expr, reasonFormat, ## __VA_ARGS__);

void assert_func(bool result, const char *pred, const char *reasonFormat, ...) {
  if (!result) {
    logf("ASSERTION FAILED: %s", pred);
    va_list argptr;
    va_start(argptr, reasonFormat);
    _logf(true, reasonFormat, argptr);
    va_end(argptr);
#if DEBUG
    while (1) delay(100);
#endif
  }
}

#define MOD_DISTANCE(a, b, m) (m / 2. - fmod((3 * m) / 2 + a - b, m))

inline int mod_wrap(int x, int m) {
  int result = x % m;
  return result < 0 ? result + m : result;
}

inline float fmod_wrap(float x, int m) {
  float result = fmodf(x, m);
  return result < 0 ? result + m : result;
}

inline unsigned int ledxy(int x, int y, int width=TOTAL_WIDTH, int height=TOTAL_HEIGHT, bool wrap=false) {
  if (wrap) x = mod_wrap(x, width);
  if (wrap) y = mod_wrap(y, height);
  // support zigzag
#if DEBUG
  if (x < 0 || x >= width) {
    logf("ledxy: OUT OF BOUNDS AT %i,%i", x,y);
    while(1) delay(100);
  } else if (y < 0 || y >= height) {
    logf("ledxy: OUT OF BOUNDS AT %i,%i", x,y);
    while(1) delay(100);
  }
#endif
  unsigned int index = 0;
  if (x & 1) {
    index = (x + 1) * height - 1 - y;
  } else {
    index= x * height + y;
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

void printColor(CRGB color) {
  loglf("CRGB(0x%x, 0x%x, 0x%x)", color.r, color.g, color.b);
}

void printColor(CHSV color) {
  loglf("CHSV(0x%x, 0x%x, 0x%x)", color.h, color.s, color.v);
}

int lsb_noise(int pin, int numbits) {
  // TODO: Use Entropy.h? Probs not needed just to randomize pattern.
  int noise = 0;
  for (int i = 0; i < numbits; ++i) {
    int val = analogRead(pin);
    noise = (noise << 1) | (val & 1);
  }
  return noise;
}

#endif
