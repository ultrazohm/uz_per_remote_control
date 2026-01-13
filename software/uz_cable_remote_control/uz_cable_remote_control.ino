/*
* Cable Remote Control for UltraZohm
*
* Januar 2026 - Michael Hoerner
*
*/

// Include libraries
#include <ezButton.h>     //Buttons
#include "driver/twai.h"  //CAN

// Activate debug outputs in serial terminal
#define DEBUG_POTIS_SWITCHES_AND_BUTTONS false
#define CAN_DEBUG false

// POTIS, BUTTONS and SWITCHES
// Define button pins and debounce time
#define SWITCH_0_PIN 2
#define SWITCH_1_PIN 3
#define SWITCH_2_PIN 4
#define SWITCH_3_PIN 5
#define BUTTON_0_PIN 6
#define BUTTON_1_PIN 7
#define BUTTON_2_PIN 8
#define BUTTON_3_PIN 9
#define DEBOUNCE_TIME_MILLISECONDS 50

// Maximum ADC values for the internal 12-bit ADCs
#define ADC_MAX_VAL_INT 4095U
#define ADC_MAX_VAL_FLOAT 4095.0f

#define FRACTIONAL_BITS 8

// Instantiate the switches and buttons
ezButton switch_0(SWITCH_0_PIN,INPUT_PULLUP);
ezButton switch_1(SWITCH_1_PIN,INPUT_PULLUP);
ezButton switch_2(SWITCH_2_PIN,INPUT_PULLUP);
ezButton switch_3(SWITCH_3_PIN,INPUT_PULLUP);
ezButton button_0(BUTTON_0_PIN,INPUT_PULLUP);
ezButton button_1(BUTTON_1_PIN,INPUT_PULLUP);
ezButton button_2(BUTTON_2_PIN,INPUT_PULLUP);
ezButton button_3(BUTTON_3_PIN,INPUT_PULLUP);

// Assign ADC pins for the potentiometers
const int pot_0_pin = A0;
const int pot_1_pin = A1;
const int pot_2_pin = A2;
const int pot_3_pin = A3;
// Variables for raw and normalized ADC readings
uint32_t pot_0_raw_val = 0U;
uint32_t pot_1_raw_val = 0U;
uint32_t pot_2_raw_val = 0U;
uint32_t pot_3_raw_val = 0U;
float pot_0_val_normalized = 0.0f;
float pot_1_val_normalized = 0.0f;
float pot_2_val_normalized = 0.0f;
float pot_3_val_normalized = 0.0f;
// Variables for fixedpoint scaling the ADC values for later CAN transmission
float pot_0_val_scaledf = 0.0f;
float pot_1_val_scaledf = 0.0f;
float pot_2_val_scaledf = 0.0f;
float pot_3_val_scaledf = 0.0f;
uint8_t pot_0_val_scaled = 0U;
uint8_t pot_1_val_scaled = 0U;
uint8_t pot_2_val_scaled = 0U;
uint8_t pot_3_val_scaled = 0U;

// Variable for the combined state of all switches and buttons
uint8_t switchesAndButtonStates = 0U;

// CAN
// Pins used to connect to CAN bus transceiver:
#define RX_PIN 44
#define TX_PIN 43
// Intervall:
#define POLLING_RATE_MS 100
#define TRANSMIT_RATE_MS 10
static bool driver_installed = false;
unsigned long previousMillis = 0;  // will store last time a message was send
uint8_t ticker = 0U;

// CAN send function
static void send_message() {
  // Send message
  ticker++;
  if(ticker > 254U) {
    ticker = 0U;
  }
  // Configure message to transmit
  twai_message_t message;

    message.extd = 0;
    message.identifier = 0x22;
    message.data_length_code = 0x06;
    message.data[0] = ticker;
    message.data[1] = switchesAndButtonStates;
    message.data[2] = pot_0_val_scaled;
    message.data[3] = pot_1_val_scaled;
    message.data[4] = pot_2_val_scaled;
    message.data[5] = pot_3_val_scaled;


  // Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    printf("Message queued for transmission\n");
  } else {
    printf("Failed to queue message for transmission\n");
  }
}


// SETUP
void setup() {
  // Start serial
  Serial.begin(115200);
  
  // Set debounce time for switches and buttons
  switch_0.setDebounceTime(DEBOUNCE_TIME_MILLISECONDS);
  switch_1.setDebounceTime(DEBOUNCE_TIME_MILLISECONDS);
  switch_2.setDebounceTime(DEBOUNCE_TIME_MILLISECONDS);
  switch_3.setDebounceTime(DEBOUNCE_TIME_MILLISECONDS);
  button_0.setDebounceTime(DEBOUNCE_TIME_MILLISECONDS);
  button_1.setDebounceTime(DEBOUNCE_TIME_MILLISECONDS);
  button_2.setDebounceTime(DEBOUNCE_TIME_MILLISECONDS);
  button_3.setDebounceTime(DEBOUNCE_TIME_MILLISECONDS);

  // Setup CAN driver
  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } else {
    Serial.println("Failed to install driver");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("Driver started");
  } else {
    Serial.println("Failed to start driver");
    return;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts");
    return;
  }

  // TWAI driver is now successfully installed and started
  driver_installed = true;

}

// CAN receive function
static void handle_rx_message(twai_message_t& message) {
  // Process received message
  if (message.extd) {
    #if CAN_DEBUG == true
    Serial.println("Message is in Extended Format");
    #endif
  } else {
    #if CAN_DEBUG == true
    Serial.println("Message is in Standard Format");
    #endif
  }
  #if CAN_DEBUG == true
  Serial.printf("ID: %x\nByte:", message.identifier);
  if (!(message.rtr)) {
    for (int i = 0; i < message.data_length_code; i++) {
      Serial.printf(" %d = %02x,", i, message.data[i]);
    }
    Serial.println("");
  }
  #endif
}


// LOOP
void loop() {
  if (!driver_installed) {
    // Driver not installed
    delay(1000);
    return;
  }

  // read analog values
  pot_0_raw_val = analogRead(pot_0_pin);
  pot_1_raw_val = analogRead(pot_1_pin);
  pot_2_raw_val = analogRead(pot_2_pin);
  pot_3_raw_val = analogRead(pot_3_pin);
  // normalize the raw ADC values to the range 0.0 ... 1.0
  pot_0_val_normalized = (ADC_MAX_VAL_INT - pot_0_raw_val) / ADC_MAX_VAL_FLOAT;
  pot_1_val_normalized = (ADC_MAX_VAL_INT - pot_1_raw_val) / ADC_MAX_VAL_FLOAT;
  pot_2_val_normalized = (ADC_MAX_VAL_INT - pot_2_raw_val) / ADC_MAX_VAL_FLOAT;
  pot_3_val_normalized = (ADC_MAX_VAL_INT - pot_3_raw_val) / ADC_MAX_VAL_FLOAT;
  // scale analog values for CAN transmission
  if(pot_0_val_normalized >= 0.99609375f) {
    pot_0_val_normalized = 0.99609375f;
  }
  pot_0_val_scaledf = ldexpf(pot_0_val_normalized, FRACTIONAL_BITS);
  pot_0_val_scaledf = roundf(pot_0_val_scaledf);
  pot_0_val_scaled = (uint8_t)pot_0_val_scaledf;

  if(pot_1_val_normalized >= 0.99609375f) {
    pot_1_val_normalized = 0.99609375f;
  }
  pot_1_val_scaledf = ldexpf(pot_1_val_normalized, FRACTIONAL_BITS);
  pot_1_val_scaledf = roundf(pot_1_val_scaledf);
  pot_1_val_scaled = (uint8_t)pot_1_val_scaledf;

  if(pot_2_val_normalized >= 0.99609375f) {
    pot_2_val_normalized = 0.99609375f;
  }
  pot_2_val_scaledf = ldexpf(pot_2_val_normalized, FRACTIONAL_BITS);
  pot_2_val_scaledf = roundf(pot_2_val_scaledf);
  pot_2_val_scaled = (uint8_t)pot_2_val_scaledf;

  if(pot_3_val_normalized >= 0.99609375f) {
    pot_3_val_normalized = 0.99609375f;
  }
  pot_3_val_scaledf = ldexpf(pot_3_val_normalized, FRACTIONAL_BITS);
  pot_3_val_scaledf = roundf(pot_3_val_scaledf);
  pot_3_val_scaled = (uint8_t)pot_3_val_scaledf;


  // update switches and button readings
  switch_0.loop();
  switch_1.loop();
  switch_2.loop();
  switch_3.loop();
  button_0.loop();
  button_1.loop();
  button_2.loop();
  button_3.loop();
  // set the respective bits in the state variable 
  if(switch_0.getState() == 0) {
    switchesAndButtonStates |= (1 << 7);
  } else {
    switchesAndButtonStates &= ~(1 << 7);
  }
  if(switch_1.getState() == 0) {
    switchesAndButtonStates |= (1 << 6);
  } else {
    switchesAndButtonStates &= ~(1 << 6);
  }
  if(switch_2.getState() == 0) {
    switchesAndButtonStates |= (1 << 5);
  } else {
    switchesAndButtonStates &= ~(1 << 5);
  }    
  if(switch_3.getState() == 0) {
    switchesAndButtonStates |= (1 << 4);
  } else {
    switchesAndButtonStates &= ~(1 << 4);
  }  
  if(button_0.getState() == 0) {
  switchesAndButtonStates |= (1 << 3);
  } else {
    switchesAndButtonStates &= ~(1 << 3);
  }
  if(button_1.getState() == 0) {
    switchesAndButtonStates |= (1 << 2);
  } else {
    switchesAndButtonStates &= ~(1 << 2);
  }
  if(button_2.getState() == 0) {
    switchesAndButtonStates |= (1 << 1);
  } else {
    switchesAndButtonStates &= ~(1 << 1);
  }    
  if(button_3.getState() == 0) {
    switchesAndButtonStates |= (1 << 0);
  } else {
    switchesAndButtonStates &= ~(1 << 0);
  } 

// CAN transmission handling
  // Check if alert happened
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
  twai_status_info_t twaistatus;
  twai_get_status_info(&twaistatus);

  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    #if CAN_DEBUG == true
    Serial.println("Alert: TWAI controller has become error passive.");
    #endif
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    #if CAN_DEBUG == true
    Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
    #endif
  }
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
    #if CAN_DEBUG == true
    Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
    Serial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
    Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
    Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
    #endif
  }

  // Check if message is received
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    // One or more messages received. Handle all.
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      handle_rx_message(message);
    }
  }

    unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
    previousMillis = currentMillis;
    send_message();
  }

  // DEBUG output
  #if DEBUG_POTIS_SWITCHES_AND_BUTTONS==true
    Serial.print("Sw_0: ");
    Serial.print(switch_0.getState());
    Serial.print("  Sw_1: ");
    Serial.print(switch_1.getState());
    Serial.print("  Sw_2: ");
    Serial.print(switch_2.getState());
    Serial.print("  Sw_3: ");
    Serial.print(switch_3.getState());

    Serial.print("\t Btn_0: ");
    Serial.print(button_0.getState());
    Serial.print("  Btn_1: ");
    Serial.print(button_1.getState());
    Serial.print("  Btn_2: ");
    Serial.print(button_2.getState());
    Serial.print("  Btn_3: ");
    Serial.print(button_3.getState());

    Serial.print("\t Pot_0: ");
    Serial.print(pot_0_val_normalized);
    Serial.print("  Pot_1: ");
    Serial.print(pot_1_val_normalized);
    Serial.print("  Pot_2: ");
    Serial.print(pot_2_val_normalized);
    Serial.print("  Pot_3: ");
    Serial.print(pot_3_val_normalized);

    Serial.print("\t Pot_0_scld: ");
    Serial.print(pot_0_val_scaledf);

    Serial.print("\t state: ");
    Serial.println(switchesAndButtonStates, BIN);
   #endif
}
