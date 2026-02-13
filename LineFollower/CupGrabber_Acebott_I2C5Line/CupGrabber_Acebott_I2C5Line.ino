#include <five_line_tracker_v3.h>

#define INFINITE_LOOP_ON_FAILURE InfiniteLoopOnFailure(__FUNCTION__, __LINE__)

#define LINE_1_DETECTED 0xf
#define LINE_2_DETECTED 0x17
#define LINE_3_DETECTED 0x1b
#define LINE_4_DETECTED 0x1d
#define LINE_5_DETECTED 0x1e
#define ALL_LINES_DETECTED 0x1f
#define NO_LINES_DETECTED 0x0

#define LINE_1_HARD_LEFT    "Hard left"
#define LINE_2_SLIGHT_LEFT  "Slight left"
#define LINE_3_CENTER       "Center"
#define LINE_4_SLIGHT_RIGHT "Slight right"
#define LINE_5_HARD_RIGHT     "Hard right"



namespace {

emakefun::FiveLineTracker g_five_line_tracker;

void InfiniteLoopOnFailure(const char* function, const uint32_t line_number) {
  Serial.println(String(F("entering an infinite loop due to failure in ")) + function + ", at line number: " + line_number);
  while (true) {
    yield();
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial.println(F("setup"));

  Wire.begin();


  const auto ret = g_five_line_tracker.Initialize();

  if (emakefun::FiveLineTracker::kOK == ret) {
    Serial.println(F("five line tracker initialization successful"));
  } else {
    Serial.print(F("five line tracker initialization failed: "));
    Serial.println(ret);
    INFINITE_LOOP_ON_FAILURE;
  }

  Serial.println(String(F("five line tracker firmware version: ")) +
                 static_cast<uint32_t>(g_five_line_tracker.FirmwareVersion()));

  const uint16_t high_thresholds[emakefun::FiveLineTracker::kLineNumber] = {850, 850, 850, 850, 850};
  g_five_line_tracker.HighThresholds(high_thresholds);

  const uint16_t low_thresholds[emakefun::FiveLineTracker::kLineNumber] = {800, 800, 800, 800, 800};
  g_five_line_tracker.LowThresholds(low_thresholds);

  Serial.println(F("setup successful"));
}

void loop() {

  uint16_t analog_values[emakefun::FiveLineTracker::kLineNumber] = {0};
  g_five_line_tracker.AnalogValues(analog_values);

  const uint8_t digital_values = g_five_line_tracker.DigitalValues();

  //printAnalogAndDigitalValues(analog_values, digital_values);
  printDigitalValueSingleLine(digital_values);
  delay(250);

}

void printDigitalValueSingleLine(const uint8_t digital_values){

  if(digital_values == LINE_1_DETECTED){
      Serial.println(LINE_1_HARD_LEFT);
      return;
  }

  if(digital_values == LINE_2_DETECTED){
      Serial.println(LINE_2_SLIGHT_LEFT);
      return;
  }

  if(digital_values == LINE_3_DETECTED){
      Serial.println(LINE_3_CENTER);
      return;
  }

  if(digital_values == LINE_4_DETECTED){
      Serial.println(LINE_4_SLIGHT_RIGHT);
      return;
  }

  if(digital_values == LINE_5_DETECTED){
      Serial.println(LINE_5_HARD_RIGHT);
      return;
  }

  if(digital_values == NO_LINES_DETECTED){
      Serial.println("No line detected");
      return;
  }
  
  if(digital_values == ALL_LINES_DETECTED){
      Serial.println("All line detected");
      return;
  }

  // Everything else
  Serial.println("Unhandled case of multiple lines detected");

}


// Original example code block extracted to this function.
void printAnalogAndDigitalValues(uint16_t analog_values[5], const uint8_t digital_values){
  String log(F("analog values: "));
  for (uint8_t i = 0; i < emakefun::FiveLineTracker::kLineNumber; i++) {
    log += i;
    log += F(":");
    log += analog_values[i];  // 获取一路传感器的值加到日志log变量中
    log += F(", ");
  }

  log += F("digital values: 0x");
  log += String(digital_values, HEX);
  Serial.println(log);
}
