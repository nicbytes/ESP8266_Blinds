#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <arduino_homekit_server.h>
#include <NewPing.h>

#define PIN_ENA D0
#define PIN_IN1 D3
#define PIN_IN2 D4
#define PIN_US_TOP_TRIG D1
#define PIN_US_TOP_ECHO D2
#define PIN_US_BOT_TRIG D5
#define PIN_US_BOT_ECHO D6

#define MAX_DISTANCE 500

#define TIMEOUT 10000

#define SIMPLE_INFO(fmt, ...)   printf_P(PSTR(fmt "\n") , ##__VA_ARGS__);

NewPing top_sonar(PIN_US_TOP_TRIG, PIN_US_TOP_ECHO, MAX_DISTANCE);
NewPing bot_sonar(PIN_US_BOT_TRIG, PIN_US_BOT_ECHO, MAX_DISTANCE);

const char *ssid = "Kings Landing";
const char *password = "HailStone56";

extern "C" homekit_server_config_t config;
extern "C" homekit_characteristic_t name;
extern "C" void accessory_init();
extern "C" homekit_characteristic_t current_position;
extern "C" homekit_characteristic_t target_position;
extern "C" homekit_characteristic_t position_state;

enum class ProgramState {
  Calibrate,
  Run,
};
enum class MeasuredState;
class RawSonars;

long time_up = 0;
long time_down = 0;
long time_position = 0;
double proxy_current_position = 0;
ProgramState program_state = ProgramState::Calibrate;

RawSonars raw_measure();
MeasuredState approximate(RawSonars&);
MeasuredState approximateWithMeasureWithTimeout(long);
MeasuredState approximateWithMeasure();
void motor_controller_stop();
void motor_controller_up();
void motor_controller_down();
void calibrate_loop();

enum class MeasuredState {
    BeyondTop,
    Middle,
    BeyondBot,
    Error
};

class RawSonars {
public:
  int t_cm;
  int b_cm;
  RawSonars(){}
  ~RawSonars(){}
  RawSonars(const RawSonars & rhs) {
    t_cm = rhs.t_cm;
    b_cm = rhs.b_cm;
  }
  RawSonars& operator=(const RawSonars &rhs) {
    t_cm = rhs.t_cm;
    b_cm = rhs.b_cm;
    return *this;
  }
};


void homekit_setup() {
  accessory_init();
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.macAddress(mac);
  int name_len = snprintf(NULL, 0, "%s_%02X%02X%02X",
      name.value.string_value, mac[3], mac[4], mac[5]);
  char *name_value = (char*) malloc(name_len + 1);
  snprintf(name_value, name_len + 1, "%s_%02X%02X%02X",
      name.value.string_value, mac[3], mac[4], mac[5]);
  name.value = HOMEKIT_STRING_CPP(name_value);

  arduino_homekit_setup(&config);
}

void homekit_loop() {
  arduino_homekit_loop();
  static uint32_t next_heap_millis = 0;
  uint32_t time = millis();
  if (time > next_heap_millis) {
    SIMPLE_INFO("heap: %d, sockets: %d",
        ESP.getFreeHeap(), arduino_homekit_connected_clients_count());
    next_heap_millis = time + 5000;
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  
  pinMode(PIN_US_TOP_TRIG, OUTPUT);
  pinMode(PIN_US_TOP_ECHO, INPUT);
  pinMode(PIN_US_BOT_TRIG, OUTPUT);
  pinMode(PIN_US_BOT_ECHO, INPUT);

  Serial.begin(115200);
  Serial.setRxBufferSize(32);
  Serial.setDebugOutput(false);
  Serial.println("yo");

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.disconnect(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);

  SIMPLE_INFO("");
  SIMPLE_INFO("SketchSize: %d", ESP.getSketchSize());
  SIMPLE_INFO("FreeSketchSpace: %d", ESP.getFreeSketchSpace());
  SIMPLE_INFO("FlashChipSize: %d", ESP.getFlashChipSize());
  SIMPLE_INFO("FlashChipRealSize: %d", ESP.getFlashChipRealSize());
  SIMPLE_INFO("FlashChipSpeed: %d", ESP.getFlashChipSpeed());
  SIMPLE_INFO("SdkVersion: %s", ESP.getSdkVersion());
  SIMPLE_INFO("FullVersion: %s", ESP.getFullVersion().c_str());
  SIMPLE_INFO("CpuFreq: %dMHz", ESP.getCpuFreqMHz());
  SIMPLE_INFO("FreeHeap: %d", ESP.getFreeHeap());
  SIMPLE_INFO("ResetInfo: %s", ESP.getResetInfo().c_str());
  SIMPLE_INFO("ResetReason: %s", ESP.getResetReason().c_str());
  INFO_HEAP();
  homekit_setup();
  INFO_HEAP();

}

void loop() {
  homekit_loop();

  switch (program_state) {
    case ProgramState::Calibrate:
      calibrate_loop();
      break;
    case ProgramState::Run:
      run_loop();
      break;
  }
//  if (target_position.value.uint8_value != current_position.value.uint8_value) {
//    Serial.println("Adjusting");
//  }
}

void motor_controller_stop() {
  digitalWrite(PIN_ENA, LOW);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
}

void motor_controller_up() {
  digitalWrite(PIN_ENA, HIGH);
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
}

void motor_controller_down() {
  digitalWrite(PIN_ENA, HIGH);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
}

RawSonars raw_measure() {
  RawSonars raw;
  raw.t_cm = top_sonar.ping_cm();
  raw.b_cm = bot_sonar.ping_cm();
  return raw;
}

MeasuredState approximate(RawSonars& raw) {
  bool t = raw.t_cm < 10; // true if detected
  bool b = raw.b_cm < 6;

  if (  t &&  b ) {
    return MeasuredState::BeyondBot;
  } else
  if (  t && !b ) {
    return MeasuredState::Middle;
  } else
  if ( !t && !b ) {
    return MeasuredState:: BeyondTop;
  } else {
    // !t b
    return MeasuredState::Error;
  }
}

MeasuredState approximateWithMeasureWithTimeout(long timeout) {
  MeasuredState s = MeasuredState::Error;
  long t = millis();

  do {
    RawSonars raw = raw_measure();
    s = approximate(raw);
    delay(10);
  } while (s == MeasuredState::Error && (millis() - t < timeout));
  return s;
}

inline MeasuredState approximateWithMeasure() {
  return approximateWithMeasureWithTimeout(1000);
}


enum class CalibrationStage {
  Start,
  FirstRaise,
  LowerToReadyPosition,
  StartRaiseTimer,
  WatchRaise,
  StopRaiseTimer,
  StartLowerTimer,
  WatchLower,
  StopLowerTimer,
  End,
  Error,
};
auto calibration_stage = CalibrationStage::Start;
long t;

/**
 * The ESP8266 has a Software Watch Dog that resets the OS if a task blocks
 * for too long. Therefore, the calibration loop is non blocking. The
 * calibration loop passes through all the states defined in CalibrationStage
 * in the order they were defined in with the exception of the error state.
 */
void calibrate_loop() {
  RawSonars raw = raw_measure();
  MeasuredState s = approximateWithMeasure();
  
  static long t;
  static int err_no;
  static long raise_time_start, raise_time_end;
  static long lower_time_start, lower_time_end;

  switch (calibration_stage) {
    
    case CalibrationStage::Start: {
      t = millis();
      if (s == MeasuredState::BeyondBot) {
        printf("Calibrating: blinds detected already down. Lifting.\n");
        motor_controller_up();
        calibration_stage = CalibrationStage::FirstRaise;
      } else if (s == MeasuredState::Error) {
        calibration_stage = CalibrationStage::Error;
        err_no = 1;
      } else {
        calibration_stage = CalibrationStage::LowerToReadyPosition;
        printf("Calibrating: blinds detected not down. Lowering.\n");
      }
    } break;
    
    case CalibrationStage::FirstRaise: {
        motor_controller_up();
      if (millis() - t > TIMEOUT) {
        motor_controller_stop();
        calibration_stage = CalibrationStage::Error;
        err_no = 2;
      }
      if (s != MeasuredState::BeyondBot) {
        printf("Calibrating: Lift complete.\n");
        calibration_stage = CalibrationStage::LowerToReadyPosition;
        t = millis();
        motor_controller_stop();
      }
    } break;
    
    case CalibrationStage::LowerToReadyPosition: {
      motor_controller_down();
      if (millis() - t > TIMEOUT) {
        motor_controller_stop();
        calibration_stage = CalibrationStage::Error;
        err_no = 3;
      }
      if (s == MeasuredState::BeyondBot) {
        motor_controller_stop();
        printf("Calibrating: blinds down. In ready position.\n");
        calibration_stage = CalibrationStage::StartRaiseTimer;
      }
    } break;
    
    case CalibrationStage::StartRaiseTimer: {
      printf("Calibrating: Starting raise timer.\n");
      raise_time_start = t = millis();
      calibration_stage = CalibrationStage::WatchRaise;
    } break;
    
    case CalibrationStage::WatchRaise: {
      motor_controller_stop();
      delay(100);
      motor_controller_up();
      delay(500);
      if (millis() - t > TIMEOUT*6) {
        motor_controller_stop();
        calibration_stage = CalibrationStage::Error;
        err_no = 4;
      }
      if (s == MeasuredState::BeyondTop) {
        motor_controller_stop();
        calibration_stage = CalibrationStage::StopRaiseTimer;
      }
    } break;
    
    case CalibrationStage::StopRaiseTimer: {
        motor_controller_stop();
        raise_time_end = millis();
        time_up = raise_time_end - raise_time_start;
        printf("Calibrating: raise timing complete: %ul\n", time_up);
        calibration_stage = CalibrationStage::StartLowerTimer;
    } break;
    
    case CalibrationStage::StartLowerTimer: {
      printf("Calibrating: Starting lower timer.\n");
      lower_time_start = t = millis();
      calibration_stage = CalibrationStage::WatchLower;
    } break;
    
    case CalibrationStage::WatchLower: {
      motor_controller_down();
      if (millis() - t > TIMEOUT) {
        motor_controller_stop();
        calibration_stage = CalibrationStage::Error;
        err_no = 5;
      }
      if (s == MeasuredState::BeyondBot) {
        motor_controller_stop();
        calibration_stage = CalibrationStage::StopLowerTimer;
      }
    } break;
    
    case CalibrationStage::StopLowerTimer: {
        motor_controller_stop();
        lower_time_end = millis();
        time_down = lower_time_end - lower_time_start;
        printf("Calibrating: lower timing complete: %ul\n", time_down);
        calibration_stage = CalibrationStage::End;
    } break;
    
    case CalibrationStage::End: {
      program_state = ProgramState::Run;
      current_position.value.uint8_value = 0;
    } break;

    /**
     * Any situation that falls into this case stays in this case.
     */
    case CalibrationStage::Error: {
      printf("Calibration error %d\n", err_no);
      motor_controller_stop();
    } break;
  }  
}

double dmap(double x, double from_low, double from_heigh, double to_low, double to_heigh) {
  return (to_heigh - to_low)*x/(from_heigh - from_low);
}

enum class MotorState {
  Down,
  Up,
  Stop
};

void run_loop() {
  static auto prev_motor_state = MotorState::Stop;
  auto& current_position_ref = current_position.value.uint8_value;
  static long t_prev = 0;
  static long t_counter = 0;
  auto measurment = approximateWithMeasure();

  // Calculate the time between last loop and this loop
  long t_now = millis();
  long t_delta = t_now - t_prev;
  t_counter += t_delta;
  if (t_now < t_prev) {
    // Overflow has occured
    t_delta = 10;
  }
  t_prev = t_now;

  if (t_counter > 5000) {
    printf("Status: { ms: %s, tp: %d, cp: %d}\n",
      prev_motor_state==MotorState::Stop? "Stop" : (prev_motor_state==MotorState::Up?"Up":"Down"),
      target_position.value.uint8_value,
      current_position_ref
    );
  }

  // Determine how the blinds are to behave.
  if (target_position.value.uint8_value < current_position_ref) {
    // ****************** GOING DOWN *************************** //
    // check if limmit is reached.
    if (measurment == MeasuredState::BeyondBot) {
      proxy_current_position = 0;
      current_position_ref = 0;
      motor_controller_stop();
      prev_motor_state = MotorState::Stop;
      goto skip; // Using goto to 'break' out of top-level if statement.
    }
    //calculate aprox position
    time_position = dmap(proxy_current_position, 0, 100, 0, time_down);
    if (prev_motor_state == MotorState::Down) {
      time_position -= t_delta;
      proxy_current_position = dmap(time_position, 0, time_down, 0, 100);
      current_position_ref = proxy_current_position;
    }
    motor_controller_down();
    prev_motor_state = MotorState::Down;
  }
  else if (target_position.value.uint8_value > current_position_ref) {
    // ****************** GOING UP ***************************** //
    // check if limmit is reached.
    if (measurment == MeasuredState::BeyondTop) {
      proxy_current_position = 100;
      current_position_ref = 100;
      motor_controller_stop();
      prev_motor_state = MotorState::Stop;
      goto skip;
    }
    //calculate aprox position
    time_position = dmap(proxy_current_position, 0, 100, 0, time_up);
    if (prev_motor_state == MotorState::Up) {
      time_position += t_delta;
      proxy_current_position = dmap(time_position, 0, 100, 0, time_up);
      current_position_ref = proxy_current_position;
    }
    motor_controller_up();
    prev_motor_state = MotorState::Up;
  }
  else {
    motor_controller_stop();
  }
  skip:

  // Guarentee a loop delay of 10 milliseconds.
  delay(10);
}
