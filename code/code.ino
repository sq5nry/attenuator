#include <Math.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32RotaryEncoder.h>
#include "FS.h"
#include <LittleFS.h>

/*
  user's configuration
*/
volatile bool BLUE_LED_DEBUG = true;
volatile bool SERIAL_DEBUG = false;
const bool SERIAL_DEBUG_DISABLED = false;   //force no logging even if USB serial connected
const char* CAL_FILES_DIR = "/";            //default is root directory
const int ATT_INCREMENT_NORMAL = 1;
const int FREQ_INCREMENT_NORMAL = 1;
const int ATT_INCREMENT_FAST = 10;
const int FREQ_INCREMENT_FAST = 100;

/*
  hardware configuration
*/
#define PIN_BLUE_LED 15

//physical properties of the executive attenuator
#define MAX_ATTENUATION 127
#define MIN_ATTENUATION 0
#define MAX_FREQUENCY 3000
#define MIN_FREQUENCY 1

//attenuator GPO to relays wiring 1/2/4/8/16/32/64dB
const uint8_t ATT_SW[7] = {0, 20, 9, 19, 18, 1, 14};

//display
#define SCREEN_SDA 6
#define SCREEN_SCL 7
#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)

//rotary encoder
const uint8_t DI_ENCODER_A = 4;
const uint8_t DI_ENCODER_B = 3;

//switches
const uint8_t FREQ_ATT_SWITCH = 5;
const uint8_t FAST_SWITCH = 2;

//autosave
const uint32_t JOB_PERIOD = 60000;  // 60 seconds


/*
  global variables
*/
//attenuator state
volatile int attenuation, frequency, rawAttenuation;
volatile float actualAttenuation;
int storedAttenuation, storedFrequency;
float storedActualAttenuation;

struct CalibrationSet* CAL;
int CAL_SIZE = 0;

Preferences preferences;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
RotaryEncoder rotaryEncoder(DI_ENCODER_A, DI_ENCODER_B);

// Used by the `loop()` to know when to
// fire an event when the knob is turned
volatile bool turnedRightFlag = false;
volatile bool turnedLeftFlag = false;


/*
  types
*/
struct CalibrationSet { //TODO add CalibrationFile ref
  int startFreq;
  int endFreq;
  int attenuation;
  float *points;
};

struct CalibrationFile {
  char *name;
  int freqStart;
  int freqEnd;
  int att;
};

struct AttSetting {
  int requestedAtt;   //requested attenuation
  int attToSet;       //attenuator setting for best match
  float estimatedAtt; //effective attenuation, may differ from the requested by 1dB max
};

/* 
  credits: https://gist.github.com/arcao
*/
class StringStream : public Stream {
public:
    StringStream(String *s) : string(s), position(0) { }

    // Stream methods
    virtual int available() { return string->length() - position; }
    virtual int read() { return position < string->length() ? (*string)[position++] : -1; }
    virtual int peek() { return position < string->length() ? (*string)[position] : -1; }
    virtual void flush() { };
    // Print methods
    virtual size_t write(uint8_t c) { (*string) += (char)c; return 1;};

private:
    String *string;
    unsigned int length;
    unsigned int position;
};

const float INVALID_ATTENUATION = -1.0f;

/*
  program start, setup
*/
void setup() {
  pinMode(PIN_BLUE_LED, OUTPUT);
  blinkDiagnosticLed(1);

  /* display */
  Wire.begin(SCREEN_SDA, SCREEN_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    for(;;);
  }

  blinkDiagnosticLed(2);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("initializing...");
  display.setCursor(0, 8);

  if (!SERIAL_DEBUG_DISABLED && (SERIAL_DEBUG = Serial.isPlugged())) {
    display.println("USB CDC plugged in");
  } else {
    display.println("USB not plugged in");
  }
  display.setCursor(0, 16);
  display.display();

  blinkDiagnosticLed(3);

  if (SERIAL_DEBUG) {
    Serial.begin(921600);
    while (!Serial) {
      // wait for serial port to connect. Needed for native USB
      blinkDiagnosticLed(4);
    }
  }

  setupCalibration();

  /* recall stored attenuator state */
  preferences.begin("attenuator", false);
  attenuation = preferences.getUInt("att", 0);
  frequency = preferences.getUInt("freq", 2400);
  actualAttenuation = preferences.getFloat("actAtt", 0);

  blinkDiagnosticLed(4);

  /* rotary encoder */
  rotaryEncoder.setEncoderType(EncoderType::FLOATING);
  rotaryEncoder.setBoundaries(-1, 1, false);
  rotaryEncoder.onTurned(&knobCallback);
  rotaryEncoder.begin();

  blinkDiagnosticLed(5);

  /* attenuator switch */
  for (int i=0; i<7; i++) {
    pinMode(ATT_SW[i], OUTPUT);
  }
  pinMode(FREQ_ATT_SWITCH, INPUT_PULLUP);
  pinMode(FAST_SWITCH, INPUT_PULLUP);

  /* finally */
  setAttenuator(attenuation, frequency);
  displayState();

  if (SERIAL_DEBUG) Serial.println("ready");
}

void onEncoderTurned() {
  setAttenuator(attenuation, frequency);
  displayState();
}

void turnedRight(bool isFast) {
  if (digitalRead(FREQ_ATT_SWITCH)) {
    if (attenuation < MAX_ATTENUATION) {
      attenuation += isFast ? ATT_INCREMENT_FAST : ATT_INCREMENT_NORMAL;
    }
  } else {
    if (frequency < MAX_FREQUENCY) {
      frequency += isFast ? FREQ_INCREMENT_FAST : FREQ_INCREMENT_NORMAL;
    }
  }
  
	turnedRightFlag = false;
  onEncoderTurned();
}

void turnedLeft(bool isFast) {
  if (digitalRead(FREQ_ATT_SWITCH)) {
    if (attenuation > MIN_ATTENUATION) {
      attenuation -= isFast ? ATT_INCREMENT_FAST : ATT_INCREMENT_NORMAL;
      if (attenuation < 0) attenuation = 0;
    }
  } else {
    if (frequency >= MIN_FREQUENCY) {
      frequency -= isFast ? FREQ_INCREMENT_FAST : FREQ_INCREMENT_NORMAL;
      if (frequency < MIN_FREQUENCY) frequency = MIN_FREQUENCY;
    }
  }

	turnedLeftFlag = false;
  onEncoderTurned();
}

void knobCallback(long value) {
	if (turnedRightFlag || turnedLeftFlag)
		return;

	// Set a flag that we can look for in `loop()`
	// so that we know we have something to do
	switch (value)	{
		case 1:
	  		turnedRightFlag = true;
		break;

		case -1:
	  		turnedLeftFlag = true;
		break;
	}

	// Override the tracked value back to 0 so that
	// we can continue tracking right/left events
	rotaryEncoder.setEncoderValue(0);
}

inline float getPointValue(int frequency, int index) {
  if (isInRange(frequency, index)) {
    int adjustedIndex = frequency - CAL[index].startFreq;
    return CAL[index].points[adjustedIndex];
  }

  return INVALID_ATTENUATION;
}

inline bool isInRange(int frequency, int index) {
  return CAL[index].startFreq <= frequency && frequency <= CAL[index].endFreq;
}

/*
   main loop
*/
void loop() {
  /*
    store settings periodically
  */
  static uint32_t previousMillis;
  if (millis() - previousMillis >= JOB_PERIOD) {
      if (storedAttenuation !=  attenuation) {
        preferences.putUInt("att", attenuation);
        storedAttenuation = attenuation;
      }
      if (storedFrequency !=  frequency) {
        preferences.putUInt("freq", frequency);
        storedFrequency = frequency;
      }
      if (storedActualAttenuation !=  actualAttenuation) {
        preferences.putFloat("actAtt", actualAttenuation);
        storedActualAttenuation = actualAttenuation;
      }
      previousMillis += JOB_PERIOD;
  }

  /*
    encoder
  */
  bool isFast = digitalRead(FAST_SWITCH) == 0;
	if (turnedRightFlag) {
    turnedRight(isFast);
  } else if (turnedLeftFlag) {
		turnedLeft(isFast);
  }
}

/*
  display and debugging
*/
void displayState() {
  display.clearDisplay();
  display.setTextSize(2);

  char line_att[16];
  display.setCursor(0,0);
  if (actualAttenuation != INVALID_ATTENUATION) {
    if (actualAttenuation < 0.0f) actualAttenuation = 0.0f; //remove tiny glitches from calibration around zero
    sprintf(line_att, "  %5.1fdB", actualAttenuation);
    display.println(line_att);
  } else {
    display.println("     --");
  }

  char line_freq[16];
  sprintf(line_freq, "  %4dMHz", frequency);
  display.setCursor(0,16);
  display.println(line_freq);

  display.setTextSize(1);
  display.setCursor(0,40);

  display.print("min: ");
  float min = getPointValue(frequency, 0);
  if (min == INVALID_ATTENUATION) {
    display.println("no cal data");
  } else {
    display.print(min);
    display.println(" dB");
  }

  display.print("max: ");
  float max = getPointValue(frequency, CAL_SIZE-1);
  if (max == INVALID_ATTENUATION) {
    display.println("no cal data");
  } else {
    display.print(max);
    display.println(" dB");
  }

  display.print("req: ");
  display.print(attenuation);
  display.println(" dB");

  display.display();
}

void blinkDiagnosticLed(int times) {
  if (!BLUE_LED_DEBUG) return;

  for(int i=1; i<times; i++) {
    digitalWrite(PIN_BLUE_LED, 1);
    delay(100);
    digitalWrite(PIN_BLUE_LED, 0);
    delay(30);
  }
  delay(300);
}

void setAttenuator(int attenuation, int frequency) {
  AttSetting result = calculateAtt(attenuation, frequency);
  actualAttenuation = result.estimatedAtt;
  rawAttenuation = result.attToSet;
  for (int i=0; i<7; i++) {
    if (result.attToSet & (1<<i))
      digitalWrite(ATT_SW[i], HIGH);
    else
      digitalWrite(ATT_SW[i], LOW);
  }
}

/*
 calibration and file handling routines
*/

void setupCalibration() {
  /* calibration data files */
  if(!LittleFS.begin()) {
    if (SERIAL_DEBUG) Serial.println("error mounting LittleFS");
    return;
  }

  if (SERIAL_DEBUG) Serial.printf("total bytes: %u, used bytes: %u\n", LittleFS.totalBytes(), LittleFS.usedBytes());

  display.println("listing cal files");
  display.display();

  CalibrationFile* calFiles = getCalibrationFiles(LittleFS, CAL_FILES_DIR);
  CAL = (CalibrationSet*) malloc(CAL_SIZE * sizeof(CalibrationSet));
  if (CAL == NULL) {
    if (SERIAL_DEBUG) Serial.println("setup: could not allocate CAL buffer");
    return;
  }

  display.print("found ");
  display.print(CAL_SIZE);
  display.println(" calibration data files");
  display.display();

  if (SERIAL_DEBUG) Serial.printf("found %d calibration files\r\n", CAL_SIZE);
  for(int c=0; c < CAL_SIZE; c++) {
    if (SERIAL_DEBUG) Serial.printf("before read: %d dB, [%d-%d]MHz, %s\r\n", calFiles[c].att, calFiles[c].freqStart, calFiles[c].freqEnd, calFiles[c].name);
    CAL[c] = {
      calFiles[c].freqStart, 
      calFiles[c].freqEnd, 
      calFiles[c].att, 
      readCalibrationFile(LittleFS, calFiles[c].name, calFiles[c].freqStart)
    };
  }

  bubbleSort(CAL, 0, CAL_SIZE-1);
  if (SERIAL_DEBUG) 
    for(int c=0; c<CAL_SIZE; c++) 
      Serial.printf("CAL[%d]: %d-%dMHz, %ddB\r\n", c, CAL[c].startFreq, CAL[c].endFreq, CAL[c].attenuation);
}

void bubbleSort(CalibrationSet* array, byte from, byte upTo) {  //TODO use qsort from stdlib if available
 byte swaps;  
 do {
    swaps=0;
    for(byte i=from; i < upTo; i++)
      if(array[i].attenuation > array[i+1].attenuation) {
        CalibrationSet x = array[i+1];
        array[i+1] = array[i];
        array[i] = x;
        ++swaps;
      }
  } while (swaps);
}

float* readCalibrationFile(fs::FS &fs, const char *fileName, int startFreq) {
  char* fullPath = (char*) malloc(strlen(CAL_FILES_DIR) + strlen(fileName) + 1);
  if (fullPath == NULL) {
    if (SERIAL_DEBUG) Serial.println("readCalibrationFile: could not allocate buffer");
    return NULL;
  }
  strcpy(fullPath, CAL_FILES_DIR);
  strcat(fullPath, fileName);

  float* att = (float*) malloc(sizeof(float) * 1001);
  if (att == NULL) {
    if (SERIAL_DEBUG) Serial.println("readCalibrationFile: could not allocate buffer");
    return NULL;
  }
  
  if (SERIAL_DEBUG) Serial.println("readCalibrationFile: reading from " + String(fullPath));

  File file = fs.open(fullPath);
  if (!file || file.isDirectory()) {
    if (SERIAL_DEBUG) Serial.println("readCalibrationFile: failed to open file for reading");
    return NULL;
  }

  display.print(".");
  display.display();

  int lines = -1;
  String comment;
  while (file.available()) {
    if ('!' == file.peek()) { // skip s1p comments
      comment = file.readStringUntil('\n');
      if (SERIAL_DEBUG) Serial.println(comment);
      continue;
    }

    int a = file.parseFloat(SKIP_WHITESPACE);           // frequency MHz
    file.parseFloat(SKIP_WHITESPACE);                   // angle, not needed
    float c = file.parseFloat(SKIP_WHITESPACE) * -1.0f; // attenuation
    lines++;
    att[a - startFreq] = c;
  }
  file.close();

  if (SERIAL_DEBUG) Serial.printf("readCalibrationFile: read %d calibration points, addr=%p\r\n", lines, att);
  return att;
}

struct CalibrationFile* getCalibrationFiles(fs::FS &fs, const char *dirname) {
  int count = 0;
  if (SERIAL_DEBUG) Serial.printf("listing directory: '%s'\r\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    if (SERIAL_DEBUG) Serial.printf("failed to open directory '%s'\r\n", dirname);
    return NULL;
  }
  if (!root.isDirectory()) {
    if (SERIAL_DEBUG) Serial.printf("not a directory '%s'\r\n", dirname);
    return NULL;
  }

  File file = root.openNextFile();
  while (file) {
    if (SERIAL_DEBUG) {
      Serial.print("  file: ");
      Serial.print(file.name());
      Serial.print("\tsize: ");
      Serial.println(file.size());
    }

    count++;
    file = root.openNextFile();
  }
  root.close();

  CAL_SIZE = count;
  CalibrationFile* calFiles = (CalibrationFile*) malloc(sizeof(CalibrationFile) * count);
  if (calFiles == NULL) {
    if (SERIAL_DEBUG) Serial.println("could not allocate calFiles buffer");
    return NULL;
  }

  root = fs.open(dirname);
  file = root.openNextFile();
  int c=0;
  while (file) {
    char* rawName = strdup(file.name());
    calFiles[c].name = rawName;
    
    String fname = String(calFiles[c].name);
    fname.replace("-", " ");
    fname.replace(".s1p", "");
    if (SERIAL_DEBUG) Serial.printf("decoding from name: %s: ", file.name());

    StringStream stream(&fname);
    int freqStart = stream.parseInt(SKIP_WHITESPACE);
    int freqEnd = stream.parseInt(SKIP_WHITESPACE);
    int att = stream.parseInt(SKIP_WHITESPACE);
    if (SERIAL_DEBUG) Serial.printf("from/to/att: %d %d %d\r\n", freqStart, freqEnd, att);
    calFiles[c].freqStart = freqStart;
    calFiles[c].freqEnd = freqEnd;
    calFiles[c].att = att;
    if (SERIAL_DEBUG) Serial.printf("inserting to calFiles at index %d: [s=%d,e=%d,a=%d,n=%s]\r\n", c, calFiles[c].freqStart, calFiles[c].freqEnd, calFiles[c].att, calFiles[c].name);

    c++;
    file = root.openNextFile();
  }

  return calFiles;
}

/* 
 calculate attenuation by frequency and calibration
*/
AttSetting calculateAtt(int requestedAtt, int frequency) {
  if (SERIAL_DEBUG) Serial.printf("requested: %d dB @ %d MHz... ", requestedAtt, frequency);
  struct AttSetting result;
  result.requestedAtt = requestedAtt;

  //find lower calibration bank, ie. having actual attenuation higher or equal to the requested attenuation
  int lowerCalibrationBankIndex = -1;
  int higherCalibrationBankIndex= -1;
  for (int c=0; c < CAL_SIZE; c++) {
    if (SERIAL_DEBUG) Serial.printf("\r\nL inRange %d, frequency=%d, c=%d, CAL[c].startFreq=%d, CAL[c].point=%4.2f\r\n", isInRange(frequency, c), frequency, c, CAL[c].startFreq, getPointValue(frequency, c));
    if (!isInRange(frequency, c)) continue;
    if (getPointValue(frequency, c) <= requestedAtt) {
        lowerCalibrationBankIndex = c;
    } else break; //previous index was the best, abandon search
  }
  for (int c=CAL_SIZE-1; c >= 0; c--) {
    if (SERIAL_DEBUG) Serial.printf("\r\nH inRange %d, frequency=%d, c=%d, CAL[c].startFreq=%d, CAL[c].point=%4.2f\r\n", isInRange(frequency, c), frequency, c, CAL[c].startFreq, getPointValue(frequency, c));
    if (!isInRange(frequency, c)) continue;   
    if (getPointValue(frequency, c) >= requestedAtt) {
        higherCalibrationBankIndex = c;
    } else break;
  }

  int x1, x2;
  float y1, y2;
  if (lowerCalibrationBankIndex != -1) {
    x1 = CAL[lowerCalibrationBankIndex].attenuation;
    y1 = getPointValue(frequency, lowerCalibrationBankIndex);
    if (SERIAL_DEBUG) Serial.printf(", L [%d dB]=%f dB", x1, y1);
  } else {
    result.attToSet = MIN_ATTENUATION;
    if (SERIAL_DEBUG) Serial.printf(", L not found, forecastAtt=%f", y1);
  }

  if (higherCalibrationBankIndex != -1) {
    x2 = CAL[higherCalibrationBankIndex].attenuation;
    y2 = getPointValue(frequency, higherCalibrationBankIndex);
    if (SERIAL_DEBUG) Serial.printf(", H [%d dB]=%f dB", x2, y2);
  } else {
    result.attToSet = MAX_ATTENUATION;
    if (SERIAL_DEBUG) Serial.printf(", H not found, forecastAtt=%f", y1);
  }

  //no calibration data for request
  if (lowerCalibrationBankIndex == -1 && higherCalibrationBankIndex == -1) {
    result.attToSet = MAX_ATTENUATION;
    result.estimatedAtt = INVALID_ATTENUATION;
    if (SERIAL_DEBUG) Serial.println("no calibration data for request");
    return result;
  }

  //insufficient or too much than requested
  if (lowerCalibrationBankIndex == -1) {
    result.estimatedAtt = getPointValue(frequency, higherCalibrationBankIndex);
  } else if (higherCalibrationBankIndex == -1) {
    result.estimatedAtt = getPointValue(frequency, lowerCalibrationBankIndex);
  }

  //requested attenuation found between two calibration datasets
  if (lowerCalibrationBankIndex != -1 && higherCalibrationBankIndex != -1) {
    float yy1 = requestedAtt - y1;
    const float x2x1 = x2 - x1;
    const float y2y1 = y2 - y1;
    float attenuationToSet = (yy1 * x2x1) / y2y1;
    attenuationToSet += x1;
    int intAttenuationToSet = round(attenuationToSet);
    
    float xx1 = intAttenuationToSet - x1;
    float forecastAtt = (y2y1 * xx1) / x2x1;
    forecastAtt += y1;

    if (SERIAL_DEBUG) Serial.printf(", resultAtt=%f, roundedAtt=%d, forecastAtt=%f\r\n", attenuationToSet, intAttenuationToSet, forecastAtt);
    result.attToSet = intAttenuationToSet;
    result.estimatedAtt = forecastAtt;
  } else {
    if (SERIAL_DEBUG) Serial.println();
  }

  return result;
}

