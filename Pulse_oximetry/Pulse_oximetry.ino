/*
 * This code utilizes Sparkfun MAX3010x library to read absorbed red an infrared light (photon count).
 * Additionally, code can return approximate heart rate and blood oxygen saturation (SpO2), which are
 * calculated based on the red and infrared light readings. See the literature for additional details 
 * how the computation is done.
 */

#include "MAX30105.h"

// Initialize an object of the library to read red and infrared lights
MAX30105 particleSensor;

// Initialize variables for red and Infra red light plotting
long red = 0;
long ir = 0;

// Initialize arrays and variables for heart rate computation
#define AVERAGING_WINDOW 10 // Compute average heart rate from successive beats. Apparently 4 is commonly used in some other implementations
#define WINDOW_LENGTH 20  // Number of samples to average in order to smooth the signal so that true peaks can be detected
long irReadings[WINDOW_LENGTH]; // Array to store IR readings
int index = 0;          // Index to keep track of current position in array
float sumReadings = 0;  // To update averaging
int lastIndex = 0;  // Index to keep track of the last position in array    
double HR = 0;  // Heart rate
long previousBeat = 0;  // Time when previous heart beat occurred
long timeReadings[AVERAGING_WINDOW];  // Array to store time readings
float sumTimeReadings = 0; // Sum of the time differences between successive heartbeats to compute heart beat 
int HR_index = 0; // Index to keep track of current position in timeReadings array
int lastHR_index = 0; // Index to keep track of the last position in timeReadings array  
unsigned long previousTime = 0;

// Bytes to check whether a value is a local maximum
long firstByte = 0;
long secondByte = 0;
long thirdByte = 0;

// Initialize arrays and variables for SpO2 computation. Use same averaging procedure as in heart rate calculation
long irReadingsSpo2[WINDOW_LENGTH];
long redReadingsSpo2[WINDOW_LENGTH];
long firstByteIR = 0;
long secondByteIR = 0;
long thirdByteIR = 0;
long firstByteRED= 0;
long secondByteRED = 0;
long thirdByteRED = 0;
double sumTimeReadingsRED = 0; // Sum of the time differences between successive heartbeats to compute heart beat 
double sumTimeReadingsIR = 0;
int indexSpo2 = 0;
int lastIndexSpo2 = 0; 

double sumREDPeaks = 0;
long REDpeaks[AVERAGING_WINDOW];
int indexSpo2REDmax = 0;
int lastIndexSpo2REDmax = 0; 

double sumREDValleys = 0;
long REDvalleys[AVERAGING_WINDOW];
int indexSpo2REDmin = 0;
int lastIndexSpo2REDmin = 0; 

double sumIRPeaks = 0;
long IRpeaks[AVERAGING_WINDOW];
int indexSpo2IRmax = 0;
int lastIndexSpo2IRmax = 0; 

double sumIRValleys = 0;
long IRvalleys[AVERAGING_WINDOW];
int indexSpo2IRmin = 0;
int lastIndexSpo2IRmin = 0; 

double AC_RED = 0;
double DC_RED = 0;
double AC_IR = 0;
double DC_IR = 0;

double SpO2 = 0;

// Define molecular extinction coefficients of HbO2 and RHb for theoretical
// SpO2 calculations. Assume red light wavelength 660nm and infrared 940nm
// Coefficients are from https://omlc.org/spectra/hemoglobin/summary.html.
// THESE ARE NOT USED IN CURRENT IMPLEMENTATION
#define E_HbO2_red 319.2
#define E_HbO2_ir 3226.56
#define E_RHb_red 1214
#define E_RHb_ir 693.44

void setup() {
  //Init serial
  Serial.begin(115200);
  while (!particleSensor.begin()) {
    Serial.println("MAX30102 was not found");
    delay(1000);
  }

  particleSensor.setup();  //Configure sensor. Use 6.4mA for LED drive. Hover mouse on top the function to see default values.
}

void loop() {
  // Plot the red and infrared pulse waves on the serial plotter. 
  // plotReadings(); 

  // Display heart rate and SpO2 every 3 seconds
  unsigned long currentTime = millis();
  HR = getHeartRate(particleSensor.getIR());
  SpO2 = getSpO2(particleSensor.getRed(),particleSensor.getIR());
  
  if (currentTime - previousTime >= 3000) {
    previousTime = currentTime;
    if (HR < 30) {
      Serial.println("Check that your finger is properly on the detector and wait a moment");
    }
    else {
    Serial.print("Heart rate: ");
    Serial.print(HR,1);
    Serial.print(" BPM");
    Serial.print("  SpO2: ");
    if (SpO2 < 0 || SpO2 > 100) {
      Serial.print("Invalid");
    }
    else {
    Serial.print(SpO2,2);
    Serial.print("%");
    }
    Serial.println();
    }
  }
}

// Function to plot Red and Infrared light readings
void plotReadings() {
  red = particleSensor.getRed();
  ir = particleSensor.getIR();
  Serial.print(red);
  Serial.print(" ");
  Serial.print(ir);
  Serial.println();
}

// Function to calculate the heart rate
double getHeartRate(long IRreading) {
  
  // Save read IR values
  firstByte = secondByte;
  secondByte = thirdByte;

  // Update the sum of readings. Simultaneuosly remove one of the early values from the sum so that it consideres 20 readings successive readings.
  sumReadings += (IRreading - irReadings[lastIndex]);

  // Update the irReadings array and indexes
  irReadings[index] = IRreading;
  index = (index + 1) % WINDOW_LENGTH;
  lastIndex = (lastIndex + 1) % WINDOW_LENGTH;

  // Average of readings to get more accurate maximum peak localization. This
  // can be considered as a moving average filter.
  long average = sumReadings / WINDOW_LENGTH;

  // Save the computed average of past 20 IR readings. Note that this process introduces marginal
  // delay between the actual time of the heart beat and the detected maximum peak of PPG signal.
  // However, averaging is performed to smooth signal and thus enabling accurate detection of the peaks.
  thirdByte = average;

  if ((secondByte > firstByte) && (secondByte > thirdByte)) {
    long timeStamp = millis();                // Save the timestamp when heart beat occurred
    long timeDif = timeStamp - previousBeat;  // Compute the difference between successive beats
    previousBeat = timeStamp;
    
    sumTimeReadings += (timeDif - timeReadings[lastHR_index]);
    timeReadings[HR_index] = timeDif;
    HR_index = (HR_index + 1) % AVERAGING_WINDOW;
    lastHR_index = (lastHR_index + 1) % AVERAGING_WINDOW;
  }
  HR = 60*sumTimeReadings/(AVERAGING_WINDOW*1000); // Approximate heart rate as beats per minute
  return HR;
}

// Function to calculate SpO2
double getSpO2(long REDreading, long IRreading) {

  // Save read RED and IR values
  firstByteRED = secondByteRED;
  secondByteRED = thirdByteRED;
  firstByteIR = secondByteIR;
  secondByteIR = thirdByteIR;

  // Update the sum of readings. Simultaneuosly remove one of the early values from the sum so that it consideres only 20 readings.
  sumTimeReadingsRED += (REDreading - redReadingsSpo2[lastIndexSpo2]);
  sumTimeReadingsIR += (IRreading - irReadingsSpo2[lastIndexSpo2]);

  // Update the arrays and indexes
  redReadingsSpo2[indexSpo2] = REDreading;
  irReadingsSpo2[indexSpo2] = IRreading;
  indexSpo2 = (indexSpo2 + 1) % WINDOW_LENGTH;
  lastIndexSpo2 = (lastIndexSpo2 + 1) % WINDOW_LENGTH;

  // Average of readings to get more accurate maximum peak localization. This
  // can be considered as a moving average filter.
  long averageRED = sumTimeReadingsRED / WINDOW_LENGTH;
  long averageIR = sumTimeReadingsIR / WINDOW_LENGTH;

  thirdByteRED = averageRED;
  thirdByteIR = averageIR;

  // Check if recorded reading is local maxima or minima. Save these values into array
  // to approximate AC and DC component. AC component will be the distance of the maximum 
  // and minimum averaged over 10 successive readings. DC component will be the value
  // of minimum averaged over 10 successive readings. See the literature for more details.
  if ((secondByteRED > firstByteRED) && (secondByteRED > thirdByteRED)) {
    long REDpeak = secondByteRED;
    sumREDPeaks += (REDpeak - REDpeaks[lastIndexSpo2REDmax]);
    REDpeaks[indexSpo2REDmax] = REDpeak;
    indexSpo2REDmax = (indexSpo2REDmax + 1) % AVERAGING_WINDOW;
    lastIndexSpo2REDmax = (lastIndexSpo2REDmax + 1) % AVERAGING_WINDOW;
  }
  if ((secondByteRED < firstByteRED) && (secondByteRED < thirdByteRED)) {
    long REDvalley = secondByteRED;
    sumREDValleys += (REDvalley - REDvalleys[lastIndexSpo2REDmin]);
    REDvalleys[indexSpo2REDmin] = REDvalley;
    indexSpo2REDmin = (indexSpo2REDmin + 1) % AVERAGING_WINDOW;
    lastIndexSpo2REDmin = (lastIndexSpo2REDmin + 1) % AVERAGING_WINDOW;
  }

  if ((secondByteIR > firstByteIR) && (secondByteIR > thirdByteIR)) {
    long IRpeak = secondByteIR;
    sumIRPeaks += (IRpeak - IRpeaks[lastIndexSpo2IRmax]);
    IRpeaks[indexSpo2IRmax] = IRpeak;
    indexSpo2IRmax = (indexSpo2IRmax + 1) % AVERAGING_WINDOW;
    lastIndexSpo2IRmax = (lastIndexSpo2IRmax + 1) % AVERAGING_WINDOW;
  }
  if ((secondByteIR < firstByteIR) && (secondByteIR < thirdByteIR)) {
    long IRvalley = secondByteIR;
    sumIRValleys += (IRvalley - IRvalleys[lastIndexSpo2IRmin]);
    IRvalleys[indexSpo2IRmin] = IRvalley;
    indexSpo2IRmin = (indexSpo2IRmin + 1) % AVERAGING_WINDOW;
    lastIndexSpo2IRmin = (lastIndexSpo2IRmin + 1) % AVERAGING_WINDOW;
  }

  AC_RED = (sumREDPeaks - sumREDValleys)/AVERAGING_WINDOW;
  DC_RED = sumREDValleys/AVERAGING_WINDOW;
  AC_IR = (sumIRPeaks - sumIRValleys)/AVERAGING_WINDOW;
  DC_IR = sumIRValleys/AVERAGING_WINDOW;

  // Compute the ratio of ratios
  double R = (AC_RED/DC_RED)/(AC_IR/DC_IR);

  // Compute theoretical SpO2
  //SpO2 = -E_HbO2_red - R*E_RHb_ir*(E_HbO2_red - E_RHb_red) - R*(E_HbO2_ir - E_RHb_ir);
  // SpO2 = (E_RHb_ir*R-E_RHb_ir)/((E_HbO2_red - E_RHb_red)-R*(E_HbO2_ir - E_RHb_ir)); // This is from Marika Rissanen B.Sc thesis. Assumption is that red and IR travel same length

  // Manufacturers have computed SpO2 with following equation. Assumption is that they have calibrated the sensor with clinical data. Thus, this equation will be used to approximate SpO2.
  SpO2 = -45.060*R *R + 30.354 *R + 94.845;

  return SpO2; // Return SpO2 (%)
}
