// https://github.com/maximkulkin/esp-homekit-demo/blob/master/examples/window_covering/main.c


/*
   simple_led_accessory.c
   Define the accessory in pure C language using the Macro in characteristics.h

    Created on: 2020-02-08
        Author: Mixiaoxiao (Wang Bin)
*/

#include <Arduino.h>
#include <homekit/types.h>
#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <stdio.h>
#include <port.h>

//const char * buildTime = __DATE__ " " __TIME__ " GMT";

#define ACCESSORY_NAME ("Nic's Bed Blind")
#define ACCESSORY_SN ("SN_NB_00001") //SERIAL_NUMBER
#define ACCESSORY_MANUFACTURER ("Arduino Homekit")
#define ACCESSORY_MODEL ("ESP8266")

#define POSITION_CLOSED 0
#define POSITION_STATE_OPENING 0
#define POSITION_STATE_CLOSING 1
#define POSITION_STATE_STOPPED 2

//#define D0 -
//#define D1 5
//#define D2 4
//#define D3 0
//#define D4 2
//#define D5 -
//#define D6 -
//#define D7 -
//#define D8 -
//#define D9 -

//#define PIN_TX 1
//#define PIN_RX 3
//#define PIN_BUILTIN_LED 2


void on_update_target_position(homekit_characteristic_t *ch, homekit_value_t value, void *context);

homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, ACCESSORY_NAME);
homekit_characteristic_t serial_number = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, ACCESSORY_SN);
homekit_characteristic_t occupancy_detected = HOMEKIT_CHARACTERISTIC_(OCCUPANCY_DETECTED, 0);
homekit_characteristic_t current_position = {
  HOMEKIT_DECLARE_CHARACTERISTIC_CURRENT_POSITION(50)
};

homekit_characteristic_t target_position = {
  HOMEKIT_DECLARE_CHARACTERISTIC_TARGET_POSITION(50, .callback = HOMEKIT_CHARACTERISTIC_CALLBACK(on_update_target_position))
};

homekit_characteristic_t position_state = {
  HOMEKIT_DECLARE_CHARACTERISTIC_POSITION_STATE(POSITION_STATE_STOPPED)
};


void update_blinds() {
  printf("update_blinds: t:");
  printf("%u", target_position.value.uint8_value);
  printf(", \tc:");
  printf("%u", current_position.value.int_value);
  printf(", \ts:");
  printf("%u", position_state.value.int_value);
  printf("\n");
}

void on_update_target_position(homekit_characteristic_t *ch, homekit_value_t value, void *context) {
  printf("Update target position to: %u\n", target_position.value.uint8_value);

  if (target_position.value.int_value == current_position.value.int_value) {
    printf("Current position equal to target. Stopping.\n");
    position_state.value.int_value = POSITION_STATE_STOPPED;
    homekit_characteristic_notify(&position_state, position_state.value);
    // vTaskSuspend(updateStateTask);
  } else {
    position_state.value.int_value = target_position.value.int_value > current_position.value.int_value
                                     ? POSITION_STATE_OPENING
                                     : POSITION_STATE_CLOSING;

    homekit_characteristic_notify(&position_state, position_state.value);
    // vTaskResume(updateStateTask);
    update_blinds();
  }
}

void on_update_current_position(homekit_characteristic_t *ch, homekit_value_t value, void *context) {
  printf("Update current position to: %u\n", current_position.value.uint8_value);
}


homekit_accessory_t *accessories[] =
{
  HOMEKIT_ACCESSORY(
    .id = 1,
    .category = homekit_accessory_category_window_covering,
  .services = (homekit_service_t *[]) {
    HOMEKIT_SERVICE(ACCESSORY_INFORMATION,
    .characteristics = (homekit_characteristic_t *[]) {
      &name,
      HOMEKIT_CHARACTERISTIC(MANUFACTURER, ACCESSORY_MANUFACTURER),
      &serial_number,
      HOMEKIT_CHARACTERISTIC(MODEL, ACCESSORY_MODEL),
      HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.0.9"),
      // HOMEKIT_CHARACTERISTIC(IDENTIFY, accessory_identify),
      NULL
    }),
    HOMEKIT_SERVICE(WINDOW_COVERING,
                    .primary = true,
    .characteristics = (homekit_characteristic_t *[]) {
      HOMEKIT_CHARACTERISTIC(NAME, "Bed Blind"),
                             &current_position,
                             &target_position,
                             &position_state,
                             NULL
    }),
    NULL
  }),
  NULL
};

homekit_server_config_t config = {
  .accessories = accessories,
  .password = "111-11-111",
  //.on_event = on_homekit_event,
  .setupId = "ABCD"
};

void accessory_init()
{
  // pinMode(PIN_LED, OUTPUT);
  // led_update();
}
