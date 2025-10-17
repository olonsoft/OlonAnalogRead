#include <Arduino.h>
#include "AnalogReader.h"

Olon::AnalogReader aReader;

uint32_t lastTime = 0;

void setup() {
  Serial.begin(115200);

  aReader.setPin(A0);
  aReader.setSampleCount(10);
  aReader.setSampleDelay(10); // msecs
  //aReader.setMaxAnalogReading(1023); // 1023 for ESP8622
  aReader.setMaxVoltage(5.2); // the voltage @ maxAnalogReading
  aReader.onComplete([](){
    Serial.println(aReader.voltage(), 3);  // Print with 3 decimal places
  });

  aReader.run();
}

void loop() {
  if (millis() - lastTime > 2000 && !aReader.busy()) {
    lastTime = millis();
    aReader.run();
  }
  aReader.loop();
}

/*
const int analogPin = A0; // Analog pin connected to the voltage divider
float voltageDividerRatio = 1.36; // Adjust according to your resistors
const int numSamples = 11; // For median and outlier methods
const float maxAnalogReading = 1023.0; // 10-bit ADC
const float referenceVoltage = 3.3; // ESP8266 reference voltage (3.3V)
const float threshold = 0.05; // 5% deviation threshold for outlier rejection
const float alpha = 0.1; // Smoothing factor for EMA
int samples[numSamples];
float lastEMA = 0; // Initialize EMA with the first reading

float readBatteryVoltageMedian() {
  // Take multiple samples
  for (int i = 0; i < numSamples; i++) {
    samples[i] = analogRead(analogPin);
    delay(5); // Small delay between readings
  }

  // Sort the samples
  for (int i = 0; i < numSamples - 1; i++) {
    for (int j = i + 1; j < numSamples; j++) {
      if (samples[i] > samples[j]) {
        int temp = samples[i];
        samples[i] = samples[j];
        samples[j] = temp;
      }
    }
  }

  // Get the median value
  int medianValue = samples[numSamples / 2];

  // Convert the median analog value to voltage
  float batteryVoltage = (medianValue / maxAnalogReading) * referenceVoltage * voltageDividerRatio;
  return batteryVoltage;
}

float readBatteryVoltageWithRejection() {
  float sampleSum = 0;
  int validSamples = 0;
  float firstSamplesMean = 0;
  const int initialSamples = 3;  // Take an initial set of samples to calculate a better mean

  // Collect initial samples to establish a baseline
  for (int i = 0; i < initialSamples; i++) {
    float initialSample = analogRead(analogPin);
    sampleSum += initialSample;
    delay(5); // Small delay between readings
  }

  // Calculate the initial mean
  firstSamplesMean = sampleSum / initialSamples;
  sampleSum = 0;  // Reset sum for the real sampling

  // Now sample the remaining values and reject outliers
  for (int i = 0; i < numSamples; i++) {
    float currentSample = analogRead(analogPin);

    // Reject samples that deviate too much from the mean
    if (abs(currentSample - firstSamplesMean) / firstSamplesMean < threshold) {
      sampleSum += currentSample;
      validSamples++;
    }

    delay(5); // Small delay between readings
  }

  // If no valid samples were found, fall back to the initial mean
  float finalMean = (validSamples > 0) ? (sampleSum / validSamples) : firstSamplesMean;

  // Convert the average valid samples to voltage
  float batteryVoltage = (finalMean / maxAnalogReading) * referenceVoltage * voltageDividerRatio;
  return batteryVoltage;
}

float readBatteryVoltageEMA() {
  // Read current value
  float currentReading = analogRead(analogPin);

  // Apply EMA formula: EMA = alpha * currentValue + (1 - alpha) * lastEMA
  float ema = alpha * currentReading + (1 - alpha) * lastEMA;

  // Update last EMA for the next iteration
  lastEMA = ema;

  // Convert the EMA value to voltage
  float batteryVoltage = (ema / maxAnalogReading) * referenceVoltage * voltageDividerRatio;
  return batteryVoltage;
}

void setup() {
  Serial.begin(115200);
  lastEMA = analogRead(analogPin); // Initialize EMA with the first reading
}

void loop() {
  Serial.print("Voltage divider ratio: ");
  Serial.println(voltageDividerRatio);
  float batteryVoltageMedian = readBatteryVoltageMedian();
  float batteryVoltageRejection = readBatteryVoltageWithRejection();
  float batteryVoltageEMA = readBatteryVoltageEMA();

  Serial.print("Median Filter Voltage: ");
  Serial.println(batteryVoltageMedian, 2); // Print with 2 decimal places

  Serial.print("Outlier Rejection Voltage: ");
  Serial.println(batteryVoltageRejection, 2);

  Serial.print("Exponential Moving Average Voltage: ");
  Serial.println(batteryVoltageEMA, 2);

  Serial.println(); // Blank line for readability
  delay(2000); // Delay between readings
  voltageDividerRatio += 0.005f;
  if (voltageDividerRatio > 1.4f) voltageDividerRatio = 1.3f;
}
*/