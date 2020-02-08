
// TODO: make into a non-Pattern manager class that patterns can subscribe to


#ifndef AUDIO_MANAGER_H
#define AUDIO_MANAGER_H

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include "patterns.h"

// GUItool: begin automatically generated code
AudioInputI2S            i2s1;           //xy=86.55555725097656,246.33336782455444
AudioMixer4              mixer1;         //xy=238.88888888888889,258.88888888888886
AudioAmplifier           amp1;           //xy=389.7777690887451,260.777774810791
AudioAnalyzePeak         peak1;          //xy=514.2222290039062,297.0000171661377
AudioAnalyzeFFT1024      fft1024_1;      //xy=521.6666984558105,210.6666717529297
AudioConnection          patchCord1(i2s1, 0, mixer1, 0);
AudioConnection          patchCord2(i2s1, 1, mixer1, 1);
AudioConnection          patchCord3(mixer1, amp1);
AudioConnection          patchCord4(amp1, fft1024_1);
AudioConnection          patchCord5(amp1, peak1);
// GUItool: end automatically generated code
    
class Sound: public Pattern {
  private:
    float gain = 10.0; // FIXME: make this dynamic

    CRGBPalette16 palette;
    bool usePalette;
    float avg = 0;
    int lead = 0;
    float offset = 0;
    
#define BAND_COUNT PANEL_WIDTH + 2 // FIXME: for some reason the first two fft bins are bogus?
    float bands[BAND_COUNT];
    int fftBinSizes[BAND_COUNT];
    const int totalBins = 512;
  public:
    Sound() {
      AudioMemory(10);
      fft1024_1.windowFunction(AudioWindowHanning1024);
    }

    void setup() {
      usePalette = (random(3) > 0);
      if (usePalette) {
        palette = gGradientPalettes[random16(gGradientPaletteCount)];
      }
      bzero(bands, sizeof(bands));

      amp1.gain(gain);
      if (fftBinSizes[PANEL_WIDTH-1] == 0) {
        getFFTBins(BAND_COUNT, totalBins, fftBinSizes);
      }
    }
    
    void update(CRGBArray<NUM_LEDS> &leds) {
      const int bandRunningAvgCount = 5;
      if (fft1024_1.available()) {
        leds.fill_solid(CRGB::Black);

        float levels[BAND_COUNT];
        for (int i = 0; i < BAND_COUNT; i++) {
          if (i < PANEL_WIDTH - 1) {
            levels[i] = fft1024_1.read(fftBinSizes[i], fftBinSizes[i + 1] - 1);
          } else {
            levels[i] = fft1024_1.read(fftBinSizes[i], totalBins - 1);
          }
        }

        for (int x = 0; x < PANEL_WIDTH; ++x) {
          float level = levels[x+2];

          if (level >= 0.01) {
            Serial.print(level);
            Serial.print(" ");
          } else {
            Serial.print("  -  "); // don't print "0.00"
          }
      
          level *= 200;

          bands[x] = (bands[x] * bandRunningAvgCount + level) / (float)(bandRunningAvgCount + 1);

          for (int y = 0; y <= bands[x] && y < PANEL_HEIGHT; ++y) {
            char bright = min(0xFF, (bands[x] - y) * 0xFF);
            leds[ledrc(x, y)] = CHSV(20 * y, 0xFF, bright);
          }
        }
        Serial.println();
      }
      
      while (Serial.available()) {
        char c = Serial.read();
        if (c == '+') {
          gain += 5;
        }  else if (c == '-') {
          gain -= 5;
        }
        Serial.print("Gain: ");
        Serial.println(gain);
        amp1.gain(gain);
      }
      if (isStopping()) {
        stopCompleted();
      }
    }

private:
    // https://forum.pjrc.com/threads/32677-Is-there-a-logarithmic-function-for-FFT-bin-selection-for-any-given-of-bands
    float FindE(int bands, int bins) {
      float increment = 0.1, eTest, n;
      int b, count, d;
    
      for (eTest = 1; eTest < bins; eTest += increment) {     // Find E through brute force calculations
        count = 0;
        for (b = 0; b < bands; b++) {                         // Calculate full log values
          n = pow(eTest, b);
          d = int(n + 0.5);
          count += d;
        }
        if (count > bins) {     // We calculated over our last bin
          eTest -= increment;   // Revert back to previous calculation increment
          increment /= 10.0;    // Get a finer detailed calculation & increment a decimal point lower
        }
        else if (count == bins)   // We found the correct E
          return eTest;       // Return calculated E
        if (increment < 0.0000001)        // Ran out of calculations. Return previous E. Last bin will be lower than (bins-1)
          return (eTest - increment);
      }
      return 0;                 // Return error 0
    }
    
    void getFFTBins(int bands, int bins, int *fftBins) {
      float e = FindE(bands, bins);
      if (e) {
        int count = 0;
        Serial.printf("E = %4.4f\n", e);
        for (int b = 0; b < bands; b++) {
          float n = pow(e, b);
          int d = int(n + 0.5);
          Serial.printf( "%4d ", count);
          fftBins[b] = count;
          count += d - 1;
          Serial.printf( "%4d\n", count);
          ++count;
        }
      } else {
        Serial.println("Error\n");
      }
    }
    
public:
    const char *description() {
      return "Sound";
    }
};

#endif
