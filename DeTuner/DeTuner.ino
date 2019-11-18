#include <arduinoFFT.h>

#define SAMPLES 128
#define SAMPLING_FREQUENCY 1000
#define MICROPHONE_IN A0

arduinoFFT FFT = arduinoFFT();

unsigned int sampleTime = round(1000000*(1.0/SAMPLING_FREQUENCY));

double real[SAMPLES];
double imag[SAMPLES];

void setup() {
  Serial.begin(9600);
}

void loop() {
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long last = micros();

    real[i] = analogRead(MICROPHONE_IN);
    imag[i] = 0;

    while (micros() - last < sampleTime);
  }

  /*FFT*/
  FFT.Windowing(real, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(real, imag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(real, imag, SAMPLES);
  for (int i = 0; i < (SAMPLES / 2); i++) {
    Serial.println(real[i], 1);
  }

  while(1);
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
