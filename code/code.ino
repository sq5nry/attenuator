#include <Math.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// #include <Adafruit_NeoPixel.h>
#include <ESP32RotaryEncoder.h>
#include "FS.h"
#include <LittleFS.h>

volatile bool BLUE_LED_DEBUG = true;
volatile bool SERIAL_DEBUG = false;
#define PIN_BLUE_LED 15

/*
  physical properties of the executive attenuator
*/
#define MAX_ATTENUATION 127
#define MIN_ATTENUATION 0
#define MAX_FREQUENCY 3000
#define MIN_FREQUENCY 0

/* attenuator controls
   1/2/4/8/16/32/64dB
 */ 
const uint8_t ATT_SW[7] = {0, 20, 9, 19, 18, 1, 14};

/* attenuator state */ 
volatile int attenuation, frequency;
volatile float actualAttenuation;
int storedAttenuation, storedFrequency;
float storedActualAttenuation;

Preferences preferences;

/* display */
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* rotary encoder */
const int8_t  DI_ENCODER_SW  = 3;
const uint8_t DI_ENCODER_A   = 5;
const uint8_t DI_ENCODER_B   = 4;
RotaryEncoder rotaryEncoder(DI_ENCODER_A, DI_ENCODER_B);

//#define PIN_WS2812B  8   // ESP32 pin that connects to WS2812B
//#define NUM_PIXELS   1  // The number of LEDs (pixels) on WS2812B
// Adafruit_NeoPixel ws2812b(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);

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
  int requestedAtt;
  int attToSet;
  float estimatedAtt;
};

const char* CAL_FILES_DIR = "/";
struct CalibrationSet* CAL;
int CAL_SIZE = 0;

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

/*
  program start
*/
void setup() {
  pinMode(PIN_BLUE_LED, OUTPUT);
  blinkDiagnosticLed(1);

  /* display */
  Wire.begin(6, 7); // SDA, SCL
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for(;;);
  }

  blinkDiagnosticLed(2);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("initializing...");
  display.setCursor(0, 8);
  if (SERIAL_DEBUG = Serial.isPlugged()) {
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

  blinkDiagnosticLed(4);

  /* rotary encoder */
  rotaryEncoder.setEncoderType(EncoderType::FLOATING);
  rotaryEncoder.setBoundaries(-1, 1, false);
  rotaryEncoder.onTurned(&knobCallback);
  rotaryEncoder.begin();

  /* diagnostic LEDs */
  // ws2812b.begin();
  // ws2812b.show();

  blinkDiagnosticLed(5);

  /* attenuator switch */
  for (int i=0; i<7; i++) {
    pinMode(ATT_SW[i], OUTPUT);
  }
  pinMode(DI_ENCODER_SW, INPUT_PULLUP);

  /* finally */
  displayState();

  /* debug */
  //Serial.begin(921600);
  //while (!Serial);
  //delay(2000);
  //Serial.println("serial port initialized");
  
  if (SERIAL_DEBUG) Serial.println("ready");
}

const uint32_t JOB_PERIOD = 60000;  // 60 seconds

// Used by the `loop()` to know when to
// fire  an event when the knob is turned
volatile bool turnedRightFlag = false;
volatile bool turnedLeftFlag = false;

void onEncoderTurned() {
  setAttenuator(attenuation, frequency);
  displayState();
}

void turnedRight() {
  if (digitalRead(DI_ENCODER_SW)) {
    if (attenuation < MAX_ATTENUATION) {
      attenuation++;
    }
  } else {
    if (frequency < MAX_FREQUENCY) {
      frequency += 10;
    }
  }
  
	turnedRightFlag = false;
  onEncoderTurned();
}

void turnedLeft() {
  if (digitalRead(DI_ENCODER_SW)) {
    if (attenuation > MIN_ATTENUATION) {
      attenuation--;
    }
  } else {
    if (frequency > MIN_FREQUENCY) {
      frequency -= 10;
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
	if (turnedRightFlag) {
    turnedRight();
  } else if (turnedLeftFlag) {
		turnedLeft();
  }
}

/*
  display and debugging
*/
void displayState() {
  display.clearDisplay();
  display.setTextSize(2);

  char line_att[16];
  sprintf(line_att, "%5.2fdB", actualAttenuation);
  display.setCursor(0,0);
  display.println(line_att);

  char line_freq[16];
  sprintf(line_freq, "%4dMHz", frequency);
  display.setCursor(0,16);
  display.println(line_freq);

  display.setTextSize(1);
  display.setCursor(0,32);

  display.print("min: ");
  display.println(CAL[0].points[frequency]);
  display.print("max: ");
  display.println(CAL[CAL_SIZE-1].points[frequency]);
  display.print("req: ");
  display.print(attenuation);
  display.println("dB");

  display.display();
  // ws2812b.setPixelColor(0, Adafruit_NeoPixel::ColorHSV(258 * attenuation, 255, 255));
  // ws2812b.setBrightness(10);
  // ws2812b.begin();
  // ws2812b.show();
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

  display.println("reading calibration data");
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
    //String filename = "/0-1000-" + String(c*5) + ".s1p";
    //CAL[c] = {0, 1000, c*5, readCalibrationFile(LittleFS, filename.c_str())};
    if (SERIAL_DEBUG) Serial.printf("before read: %d dB, [%d-%d]MHz, %s\r\n", calFiles[c].att, calFiles[c].freqStart, calFiles[c].freqEnd, calFiles[c].name);
    CAL[c] = {0, 1000, calFiles[c].att, readCalibrationFile(LittleFS, calFiles[c].name)};
  }

  bubbleSort(CAL, 0, CAL_SIZE-1);
  if (SERIAL_DEBUG) 
    for(int c=0; c<CAL_SIZE; c++) 
      Serial.printf("CAL[%d]: %d-%dMHz, %ddB\r\n", c, CAL[c].startFreq, CAL[c].endFreq, CAL[c].attenuation);
}

void bubbleSort(CalibrationSet* array, byte from, byte upTo) {
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

float* readCalibrationFile(fs::FS &fs, const char *fileName) {
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
  while (file.available()) {
    if ('!' == file.peek()) { // skip s1p comments
      file.readStringUntil('\n');
      continue;
    }

    int a = file.parseFloat(SKIP_WHITESPACE);           // frequency MHz
    file.parseFloat(SKIP_WHITESPACE);                   // angle, not needed
    float c = file.parseFloat(SKIP_WHITESPACE) * -1.0f; // attenuation
    lines++;
    att[a] = c;
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
    if (CAL[c].points[frequency] <= requestedAtt) {
        lowerCalibrationBankIndex = c;
    } else break;
  }
  for (int c=CAL_SIZE-1; c >= 0; c--) {
    if (CAL[c].points[frequency] >= requestedAtt) {
        higherCalibrationBankIndex = c;
    } else break;
  }

  int x1, x2;
  float y1, y2;
  if (lowerCalibrationBankIndex != -1) {
    x1 = CAL[lowerCalibrationBankIndex].attenuation;
    y1 = CAL[lowerCalibrationBankIndex].points[frequency];
    if (SERIAL_DEBUG) Serial.printf(", L [%d dB]=%f dB", x1, y1);
  } else {
    result.attToSet = attenuation;
    if (SERIAL_DEBUG) Serial.printf(", L not found, forecastAtt=%f", y1);
  }

  if (higherCalibrationBankIndex != -1) {
    x2 = CAL[higherCalibrationBankIndex].attenuation;
    y2 = CAL[higherCalibrationBankIndex].points[frequency];
    if (SERIAL_DEBUG) Serial.printf(", H [%d dB]=%f dB", x2, y2);
  } else {
    result.attToSet = attenuation;
    if (SERIAL_DEBUG) Serial.printf(", H not found, forecastAtt=%f", y1);
  }

  //insufficient or too much than requested
  if (lowerCalibrationBankIndex == -1) {
    result.estimatedAtt = CAL[higherCalibrationBankIndex].points[frequency];
  } else if (higherCalibrationBankIndex == -1) {
    result.estimatedAtt = CAL[lowerCalibrationBankIndex].points[frequency];
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
    result.attToSet = attenuationToSet;
    result.estimatedAtt = forecastAtt;
  } else {
    if (SERIAL_DEBUG) Serial.println();
  }

  return result;
}
