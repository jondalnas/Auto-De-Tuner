#include <arduinoFFT.h>
#include <TimedAction.h>

#define SAMPLES 128
#define SAMPLING_FREQUENCY 8000
#define MICROPHONE_IN A0
#define CUTOFF 100

arduinoFFT FFT = arduinoFFT();

unsigned int sampleTime = round(1.0/SAMPLING_FREQUENCY);

double real[SAMPLES];
double imag[SAMPLES];

struct Freq {
  uint16_t freq;
  uint16_t ampl;
};

Freq freqs[5];
struct {
  uint16_t ampl;
  uint8_t index;
} lowestFreq;

void setup() {
  Serial.begin(9600);
}

uint8_t sampleCount = 0;
void sample() {
  real[sampleCount] = analogRead(MICROPHONE_IN)-536;
  imag[sampleCount] = 0;

  sampleCount++;

  if (sampleCount < SAMPLES) return;

  sampleCount = 0;

  /*FFT*/
  FFT.Windowing(real, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(real, imag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(real, imag, SAMPLES);
  lowestFreq.ampl = CUTOFF;
  lowestFreq.index = 0;
  for (int i = 0; i < 5; i++) {
    freqs[i] = {};
  }
  
  for (int i = 0; i < (SAMPLES / 2); i++) {
    if (real[i] > lowestFreq.ampl) {
      lowestFreq.ampl = real[i];
      freqs[lowestFreq.index] = {(i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, real[i]};
      
      for (int j = 0; j < 5; j++) {
        if (freqs[j].ampl < lowestFreq.ampl) {
          lowestFreq.ampl = freqs[j].ampl;
          lowestFreq.index = j;
        }
      }
    }
    /*Serial.print((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
    Serial.print(", ");
    Serial.println(real[i], 1);*/
  }

  /*Serial.println("========================SECTION DEVIDER========================");
  for (int i = 0; i < 5; i++) {
    Serial.print((freqs[i].freq * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
    Serial.print(", ");
    Serial.println(freqs[i].ampl, 1);
  }*/
}

float notchVariables[5];
float getNotch(double freq) {
  //Goint one octave higher means doubling the frequency

  float endResult = 0;
  float fractal = 1.0/2.0;
  double halfFreq = freq / 2.0;
  for (int i = 0; i < 5; i++) {
    endResult += sin(freq*(i+1)+notchVariables[i])*fractal*halfFreq;
    fractal /= 2.0;
  }
}

void genNotchVariables() {
  for (int i = 0; i < 5; i++) {
    notchVariables[i] = ((rand() * 2.0 / RAND_MAX) - 1) * 6.0;
  }
}

TimedAction sampleTimer = TimedAction(sampleTime, sample);
TimedAction notch = TimedAction(10000, genNotchVariables);

void loop() {
  sampleTimer.check();
  notch.check();
  
  float wave = 0;
  for (int i = 0; i < 5; i++) {
    wave += sin(millis()/1000.0*2*PI * freqs[i].freq) * freqs[i].ampl;
  }
  
    Serial.println(wave);
}
