#ifndef AUDIO_MANAGER_H
#define AUDIO_MANAGER_H

#include <Audio.h>
#include "patterns.h"

// GUItool: begin automatically generated code
AudioInputI2S            i2s1;           //xy=86.55555725097656,246.33336782455444
AudioAmplifier           amp1;           //xy=389.7777690887451,260.777774810791
AudioAnalyzePeak         peak1;          //xy=514.2222290039062,297.0000171661377
AudioAnalyzeFFT1024      fft1024_1;      //xy=521.6666984558105,210.6666717529297
AudioConnection          patchCord1(i2s1, 0, amp1, 0);
AudioConnection          patchCord2(amp1, fft1024_1);
AudioConnection          patchCord3(amp1, peak1);
// GUItool: end automatically generated code

#define RAW_FFT_BANDS 512

class FFTProcessing {
  int *fftBinSizes;
  float *levels;
  float *levelsAccum;
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
      else if (count == bins)
        return eTest;
      if (increment < 0.0000001)        // Ran out of calculations. Return previous E. Last bin will be lower than (bins-1)
        return (eTest - increment);
    }
    return 0;
  }
  
  void getFFTBins(int bands, int bins, int *fftBins) {
    const int binStartOffset = 2; // HACK: For some reason the first two FFT bins are garbage
    float e = FindE(bands + 1, bins - binStartOffset);
    if (e) {
      int count = binStartOffset;
      Serial.printf("E = %4.4f\n", e);
      for (int b = 0; b < bands; b++) {
        float n = pow(e, b+1);
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
  const int fftNumBins;
  FFTProcessing(int numBins) : fftNumBins(numBins) {
    fftBinSizes = new int[fftNumBins];
    levels = new float[fftNumBins];
    levelsAccum = new float[fftNumBins];
    bzero(levelsAccum, fftNumBins * sizeof(levelsAccum[0]));

    getFFTBins(fftNumBins, RAW_FFT_BANDS, fftBinSizes);
  }
  ~FFTProcessing() {
    delete [] fftBinSizes;
    delete [] levels;
    delete [] levelsAccum;
  }

  bool fftAvailable() {
    return fft1024_1.available();
  }

  const float *fftLevels(unsigned int avgSamples=0) {
    for (int i = 0; i < fftNumBins; i++) {
      int stopIndex = (i < fftNumBins - 1 ? fftBinSizes[i + 1] - 1 : RAW_FFT_BANDS - 1);
      levels[i] = fft1024_1.read(fftBinSizes[i], stopIndex);
      if (avgSamples != 0) {
        levelsAccum[i] = (levelsAccum[i] * avgSamples + levels[i]) / (float)(avgSamples + 1);
      }
    }
    
    return (avgSamples == 0 ? levels : levelsAccum);
  }

  void fftLog() {
    const float *levels = fftLevels();
    for (int x = 0; x < fftNumBins; ++x) {
      float level = levels[x];
      if (level >= 0.01) {
        Serial.print(level);
        Serial.print(" ");
      } else {
        Serial.print("  -  "); // don't print "0.00"
      }
      Serial.println();
    }
  }

  void fftLogRaw() {
    for (int x = 0; x < RAW_FFT_BANDS; ++x) {
      float level = fft1024_1.read(x);
      if (level >= 0.01) {
        Serial.print(level);
        Serial.print(" ");
      } else {
        Serial.print("  -  "); // don't print "0.00"
      }
      Serial.println();
    }
  }
};

class AudioManager {
private:
  float gain = 10.0; // FIXME: make this dynamic

public:
  AudioManager() {
    AudioMemory(12);
    fft1024_1.windowFunction(AudioWindowHanning1024);
    amp1.gain(gain);
  }

  void subscribe() {
    
  }
  
  void unsubscribe() {
    // FIXME: need a way to disable or pause fft, but may need to patch the audio library
  }

  void printStatistics() {
    // print a summary of the current & maximum usage
    Serial.print("CPU: ");
    Serial.print("fft=");
    Serial.print(fft1024_1.processorUsage());
    Serial.print(",");
    Serial.print(fft1024_1.processorUsageMax());
    Serial.print("  ");
    Serial.print("all audio compute=");
    Serial.print(AudioProcessorUsage());
    Serial.print(",");
    Serial.print(AudioProcessorUsageMax());
    Serial.print("    ");
    Serial.print("audio memory: ");
    Serial.print(AudioMemoryUsage());
    Serial.print(",");
    Serial.print(AudioMemoryUsageMax());
    Serial.println();
  }
};

#endif
