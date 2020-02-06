#include <utility.h>
#include <system_configuration.h>
#include <unwind-cxx.h>
#include <StandardCplusplus.h>
#include <vector>

#include <arduinoFFT.h>
#include <TimedAction.h>

//Number of samples to use per transformation
#define SAMPLES 128
//How many times per second to take a sample
#define SAMPLING_FREQUENCY 800
//The input pin
#define MICROPHONE_IN A0
//Lowest amplitude to play back
#define LOW_CUTOFF 25
//The output pin
#define SPEAKER 6

arduinoFFT FFT = arduinoFFT();

unsigned int sampleTime = round(1000 * (1.0 / SAMPLING_FREQUENCY));

double const binToFreq = SAMPLING_FREQUENCY / (SAMPLES / 2.0);

double real[SAMPLES];
double imag[SAMPLES];

float notchVariables[5];

//A class that containes all information about a frequacy to be able to play it back
class Wave {
  public:
    Wave(uint16_t freq, double amplitude) : freq(freq), amplitude(amplitude) {}
    uint16_t freq;
    double amplitude;
};

//List of all frequencies the arduino has heard
std::vector<Wave*> waves = std::vector<Wave*>();

//Generate new notch variables between -6 and 6
void genNotchVariables() {
  for (int i = 0; i < 5; i++) {
    notchVariables[i] = ((rand() * 2.0 / RAND_MAX) - 1) * 6.0;
  }
}

TimedAction updateNotch = TimedAction(1000, genNotchVariables);
TimedAction sampleTimer = TimedAction(sampleTime, sample);

void setup() {
  //Begin the serial connection for debugging
  Serial.begin(9600);

  //Set the individual pins to what they need to be
  pinMode(SPEAKER, OUTPUT);
  pinMode(MICROPHONE_IN, INPUT);

  //Reset all notch variables
  for (int i = 0; i < 5; i++) {
    notchVariables[i] = 0;
  }
}

int millisec = millis();
void loop() {
  //Pause the code untli the next sample needs to be taken
  while(millis() < (millisec + sampleTime)) delay(1);
  millisec = millis();
  sample();
  
  //Check if the notches have to be updated
  updateNotch.check();
}

uint8_t sampleCount = 0;
void sample() {
  //Read the input and prepare the variables for a Fourier transformation
  real[sampleCount] = analogRead(MICROPHONE_IN) - 533;
  imag[sampleCount] = 0;

  sampleCount++;

  //If the number of samples taken is greater than or equal to the number of samples for Fourier transformation, then continue executing code
  if (sampleCount < SAMPLES) return;
  
  sampleCount = 0;

  //Perform the Fourier transformation
  FFT.Windowing(real, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(real, imag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(real, imag, SAMPLES);

  //Clear all the old recognized frequencies
  //waves.clear();

  //Loop through all recognized frequencies and add those whos amplitudes is greater than the cutoff to the list of recognized frequencies
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
  
  //Play the frequencies back through the speaker
  tone(SPEAKER, w.freq * binToFreq);
}

//This method is depricated but should be and updated version should be used at a later time
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
