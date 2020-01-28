#include <utility.h>
#include <system_configuration.h>
#include <unwind-cxx.h>
#include <StandardCplusplus.h>
#include <vector>

#include <arduinoFFT.h>
#include <TimedAction.h>

#define SAMPLES 128
#define SAMPLING_FREQUENCY 800
#define MICROPHONE_IN A0
#define LOW_CUTOFF 25
#define SPEAKER 6

arduinoFFT FFT = arduinoFFT();

unsigned int sampleTime = round(1000 * (1.0 / SAMPLING_FREQUENCY));

double const binToFreq = SAMPLING_FREQUENCY / (SAMPLES / 2.0);

double real[SAMPLES];
double imag[SAMPLES];

float notchVariables[5];

class Wave {
  public:
    Wave(uint16_t freq, double amplitude) : freq(freq), amplitude(amplitude) {}
    uint16_t freq;
    double amplitude;
};

std::vector<Wave*> waves = std::vector<Wave*>();

void genNotchVariables() {
  for (int i = 0; i < 5; i++) {
    notchVariables[i] = ((rand() * 2.0 / RAND_MAX) - 1) * 6.0;
  }
}

TimedAction updateNotch = TimedAction(1000, genNotchVariables);
TimedAction sampleTimer = TimedAction(sampleTime, sample);

void setup() {
  Serial.begin(9600);

  pinMode(SPEAKER, OUTPUT);
  pinMode(MICROPHONE_IN, INPUT);

  for (int i = 0; i < 5; i++) {
    notchVariables[i] = 0;
  }
}

int millisec = millis();
void loop() {
  while(millis() < (millisec + sampleTime)) delay(1);
  millisec = millis();
  sample();
  
  //sampleTimer.check();
  updateNotch.check();
}

uint8_t sampleCount = 0;
void sample() {
  real[sampleCount] = analogRead(MICROPHONE_IN) - 533;
  imag[sampleCount] = 0;

  sampleCount++;

  if (sampleCount < SAMPLES) return;
  
  sampleCount = 0;

  /*FFT*/
  FFT.Windowing(real, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(real, imag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(real, imag, SAMPLES);

  //waves.clear();

  Wave w = Wave(0, 0);
  for (int i = 0; i < (SAMPLES / 2); i++) {
    double amp = real[i];

    if (amp > LOW_CUTOFF) {
      if (w.amplitude < amp) {
        w.amplitude = amp;
        w.freq = i;
      }
      
      //waves.push_back(new Wave(i, amp));
    }
  }
  
  tone(SPEAKER, w.freq * binToFreq);
}

float getNotch(double freq) {
  //Going one octave higher means doubling the frequency

  float endResult = 0;
  float fractal = 1.0 / 2.0;
  for (int i = 0; i < 5; i++) {
    endResult += sin(freq * (i + 1) * 0.004 + notchVariables[i]) * fractal;
    fractal /= 2.0;
  }

  endResult *= freq / 2.0;

  return endResult;
}
