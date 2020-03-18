#define MAGNET_SENS
bool configMode;
bool door_status = true;
bool check;
#ifdef MAGNET_SENS
bool magnet_status = true;
#endif
bool vibro = true;
bool nosleep;
bool button_flag;
bool onoff = true;
bool flag_update_transport_param;
bool flag_sendRoute_parent;
bool flag_no_present;
bool flag_nogateway_mode;
bool flag_find_parent_process;
//bool flag_fcount;
bool Ack_TL;
bool Ack_FP;
bool PRESENT_ACK;
bool send_a;
bool batt_flag;
byte conf_vibro_set = 2;
byte err_delivery_beat;
byte problem_mode_count;
volatile byte axelIntStatus = 0;
volatile byte gerkIntStatus = 0;
volatile byte magIntStatus = 0;
volatile byte buttIntStatus = 0;
int8_t int_status = 0;
uint8_t  countbatt = 0;
uint8_t batt_cap;
uint8_t old_batt_cap = 100;
int16_t myid;
int16_t mypar;
int16_t old_mypar = -1;
int16_t linkQuality;
int16_t old_linkQuality;
uint16_t batteryVoltage;
uint32_t BATT_TIME;
uint32_t SLEEP_TIME = 10800000;
uint32_t SLEEP_NOGW = 60000;
uint32_t oldmillis;
uint32_t newmillis;
uint32_t previousMillis;
uint32_t lightMillisR;
uint32_t configMillis;
uint32_t interrupt_time;
uint32_t SLEEP_TIME_W;
uint32_t axel_time;
uint32_t axel_time0;
uint32_t PIN_BUTTON_MASK;
uint32_t AXEL_INT_MASK;
uint32_t GERKON_INT_MASK;
#ifdef MAGNET_SENS
uint32_t MAGNET_INT_MASK;
#endif
float ODR_1Hz6_LP_ONLY = 1.6f;
float ODR_12Hz5 = 12.5f;
float ODR_25Hz = 25.0f;
float ODR_50Hz = 50.0f;
float ODR_100Hz = 100.0f;
float ODR_200Hz = 200.0f;

//#define MY_DEBUG

#ifndef MY_DEBUG
#define MY_DISABLED_SERIAL
#endif
#define MY_RADIO_NRF5_ESB
int16_t mtwr;
#define MY_TRANSPORT_WAIT_READY_MS (mtwr)
#define MY_NRF5_ESB_PA_LEVEL (NRF5_PA_LOW)

#include <LIS2DW12Sensor.h>
LIS2DW12Sensor *lis2;

#include <MySensors.h>

extern "C" {
#include "app_gpiote.h"
#include "nrf_gpio.h"
}
#define APP_GPIOTE_MAX_USERS 1
static app_gpiote_user_id_t m_gpiote_user_id;

#define DWS_CHILD_ID 0
#define V_SENS_CHILD_ID 1
#ifdef MAGNET_SENS
#define M_CHILD_ID 2
#endif
#define LEVEL_SENSIV_V_SENS_CHILD_ID 230
#define SIGNAL_Q_ID 250

MyMessage dwsMsg(DWS_CHILD_ID, V_TRIPPED);
#ifdef MAGNET_SENS
MyMessage mMsg(M_CHILD_ID, V_TRIPPED);
#endif
MyMessage vibroMsg(V_SENS_CHILD_ID, V_TRIPPED);
MyMessage conf_vsensMsg(LEVEL_SENSIV_V_SENS_CHILD_ID, V_VAR1);
#define SN "DOOR & WINDOW SENS"
#define SV "1.14"


void before() {
  board_Init();
  happy_init();
  delay(500);
  batteryVoltage = hwCPUVoltage();
  digitalWrite(BLUE_LED, LOW);
}


void presentation()
{
  check = sendSketchInfo(SN, SV);
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(10);
    check = sendSketchInfo(SN, SV);
    wait(10);
    _transportSM.failedUplinkTransmissions = 0;
  }
  if (check) {
    blinky(1, 1, BLUE_LED);
  } else {
    blinky(1, 1, RED_LED);
  }

  check = present(DWS_CHILD_ID, S_DOOR, "STATUS RS SENS");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(20);
    check = present(DWS_CHILD_ID, S_DOOR, "STATUS RS SENS");
    wait(20);
    _transportSM.failedUplinkTransmissions = 0;
  }
  if (check) {
    blinky(1, 1, BLUE_LED);
  } else {
    blinky(1, 1, RED_LED);
  }

  check = present(V_SENS_CHILD_ID, S_VIBRATION, "STATUS SHOCK SENS");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(30);
    check = present(V_SENS_CHILD_ID, S_VIBRATION, "STATUS SHOCK SENS");
    wait(30);
    _transportSM.failedUplinkTransmissions = 0;
  }
  if (check) {
    blinky(1, 1, BLUE_LED);
  } else {
    blinky(1, 1, RED_LED);
  }
#ifdef MAGNET_SENS
  check = present(M_CHILD_ID, S_DOOR, "ANTI-MAGNET ALARM");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(40);
    check = present(M_CHILD_ID, S_DOOR, "ANTI-MAGNET ALARM");
    wait(40);
    _transportSM.failedUplinkTransmissions = 0;
  }
  if (check) {
    blinky(1, 1, BLUE_LED);
  } else {
    blinky(1, 1, RED_LED);
  }
#endif
  check = present(SIGNAL_Q_ID, S_CUSTOM, "SIGNAL %");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(40);
    check = present(SIGNAL_Q_ID, S_CUSTOM, "SIGNAL %");
    wait(40);
    _transportSM.failedUplinkTransmissions = 0;
  }
  if (check) {
    blinky(1, 1, BLUE_LED);
  } else {
    blinky(1, 1, RED_LED);
  }

  check = present(LEVEL_SENSIV_V_SENS_CHILD_ID, S_CUSTOM, "SENS LEVEL VIBRO");
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(50);
    check = present(LEVEL_SENSIV_V_SENS_CHILD_ID, S_CUSTOM, "SENS LEVEL VIBRO");
    wait(50);
    _transportSM.failedUplinkTransmissions = 0;
  }
  if (check) {
    blinky(1, 1, BLUE_LED);
  } else {
    blinky(1, 1, RED_LED);
  }

  check = send(conf_vsensMsg.set(conf_vibro_set));
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(60);
    check = send(conf_vsensMsg.set(conf_vibro_set));
    wait(60);
    _transportSM.failedUplinkTransmissions = 0;
  }
  if (check) {
    blinky(1, 1, BLUE_LED);
  } else {
    blinky(1, 1, RED_LED);
  }
}


void setup() {
  digitalWrite(BLUE_LED, HIGH);
  config_Happy_node();
  sensors_Init();
}


void loop() {
  if (flag_update_transport_param == true) {
    update_Happy_transport();
  }
  if (flag_sendRoute_parent == true) {
    present_only_parent();
  }
  if (isTransportReady() == true) {
    if (flag_nogateway_mode == false) {
      if (flag_find_parent_process == true) {
        find_parent_process();
      }
      if (configMode == false) {
#ifdef MAGNET_SENS
        if ((axelIntStatus == AXEL_INT) || (buttIntStatus == PIN_BUTTON) || (gerkIntStatus == GERKON_INT) || (magIntStatus == MAGNET_INT)) {
#else
        if ((axelIntStatus == AXEL_INT) || (buttIntStatus == PIN_BUTTON) || (gerkIntStatus == GERKON_INT)) {
#endif
          nosleep = true;
          newmillis = millis();
          interrupt_time = newmillis - oldmillis;
          BATT_TIME = BATT_TIME - interrupt_time;
          if (BATT_TIME < 60000) {
            BATT_TIME = SLEEP_TIME;
            batteryVoltage = hwCPUVoltage();
            batt_flag = true;
          }

          if (gerkIntStatus == GERKON_INT) {
            send_Gerkon();
            axel_time = millis();
            nosleep = false;
          }
#ifdef MAGNET_SENS
          if (magIntStatus == MAGNET_INT) {
            send_Magnet();
            nosleep = false;
          }
#endif
          if (axelIntStatus == AXEL_INT) {
            if (millis() - axel_time0 >= 2000) {
              send_Axel();
              nosleep = false;
            } else {
              if (digitalRead(GERKON_INT) == LOW) {
                send_Gerkon();
                axel_time = millis();
                nosleep = false;
              }
            }
          }

          if (buttIntStatus == PIN_BUTTON) {
            if (digitalRead(PIN_BUTTON) == LOW && button_flag == false) {
              button_flag = true;
              previousMillis = millis();
              ledsOff();
            }
            if (digitalRead(PIN_BUTTON) == LOW && button_flag == true) {
              if ((millis() - previousMillis > 0) && (millis() - previousMillis <= 1750)) {
                if (millis() - lightMillisR > 70) {
                  lightMillisR = millis();
                  onoff = !onoff;
                  digitalWrite(BLUE_LED, onoff);
                }
              }
              if ((millis() - previousMillis > 1750) && (millis() - previousMillis <= 2000)) {
                ledsOff();
              }
              if ((millis() - previousMillis > 2000) && (millis() - previousMillis <= 3750)) {
                if (millis() - lightMillisR > 50) {
                  lightMillisR = millis();
                  onoff = !onoff;
                  digitalWrite(GREEN_LED, onoff);
                }
              }
              if ((millis() - previousMillis > 3750) && (millis() - previousMillis <= 4000)) {
                ledsOff();
              }
              if ((millis() - previousMillis > 4000) && (millis() - previousMillis <= 5750)) {
                if (millis() - lightMillisR > 30) {
                  lightMillisR = millis();
                  onoff = !onoff;
                  digitalWrite(RED_LED, onoff);
                }
              }
              if (millis() - previousMillis > 5750) {
                ledsOff();
              }
            }

            if (digitalRead(PIN_BUTTON) == HIGH && button_flag == true) {
              if ((millis() - previousMillis <= 1750) && (button_flag == true))
              {
                ledsOff();
                blinky(2, 2, BLUE_LED);
                button_flag = false;
                buttIntStatus = 0;
                presentation();
                nosleep = false;
              }
              if ((millis() - previousMillis > 2000) && (millis() - previousMillis <= 3750) && (button_flag == true))
              {
                ledsOff();
                blinky(2, 2, GREEN_LED);
                configMode = true;
                button_flag = false;
                configMillis = millis();
                interrupt_Init(true);
                buttIntStatus = 0;
                NRF5_ESB_startListening();
                wait(50);
              }

              if ((millis() - previousMillis > 4000) && (millis() - previousMillis <= 5750) && (button_flag == true))
              {
                ledsOff();
                blinky(3, 3, RED_LED);
                new_device();
              }

              if ((((millis() - previousMillis > 1750) && (millis() - previousMillis <= 2000)) || ((millis() - previousMillis > 3750) && (millis() - previousMillis <= 4000)) || ((millis() - previousMillis > 5750))) && (button_flag == true))
              {
                ledsOff();
                nosleep = false;
                button_flag = false;
                buttIntStatus = 0;
              }
            }
          }
        } else {
          batteryVoltage = hwCPUVoltage();
          BATT_TIME = SLEEP_TIME;
          sendBatteryStatus(true);
          nosleep = false;
        }
      } else {
        if (millis() - configMillis > 30000) {
          blinky(3, 3, GREEN_LED);
          configMode = false;
          nosleep = false;
          interrupt_Init(false);
          wait(50);
        }
      }
    } else {
      if (buttIntStatus == PIN_BUTTON) {
        if (digitalRead(PIN_BUTTON) == LOW && button_flag == false) {
          button_flag = true;
          nosleep = true;
          previousMillis = millis();
          ledsOff();
        }
        if (digitalRead(PIN_BUTTON) == LOW && button_flag == true) {
          if ((millis() - previousMillis > 0) && (millis() - previousMillis <= 1750)) {
            if (millis() - lightMillisR > 25) {
              lightMillisR = millis();
              onoff = !onoff;
              digitalWrite(GREEN_LED, onoff);
            }
          }
          if ((millis() - previousMillis > 1750) && (millis() - previousMillis <= 2000)) {
            ledsOff();
          }
          if ((millis() - previousMillis > 2000) && (millis() - previousMillis <= 4000)) {
            if (millis() - lightMillisR > 25) {
              lightMillisR = millis();
              onoff = !onoff;
              digitalWrite(RED_LED, onoff);
            }
          }
          if (millis() - previousMillis > 4000) {
            ledsOff();
          }
        }

        if (digitalRead(PIN_BUTTON) == HIGH && button_flag == true) {
          if ((millis() - previousMillis <= 1750) && (button_flag == true))
          {
            ledsOff();
            blinky(2, 2, BLUE_LED);
            button_flag = false;
            buttIntStatus = 0;
            check_parent();
            nosleep = false;
          }
          if ((millis() - previousMillis > 2000) && (millis() - previousMillis <= 4000) && (button_flag == true))
          {
            ledsOff();
            blinky(3, 3, RED_LED);
            new_device();
          }

          if ((((millis() - previousMillis > 1750) && (millis() - previousMillis <= 2000)) || ((millis() - previousMillis > 4000))) && (button_flag == true))
          {
            ledsOff();
            nosleep = false;
            button_flag = false;
            buttIntStatus = 0;
          }
        }
      } else {
        check_parent();
      }
    }
  }

  if (_transportSM.failureCounter > 0)
  {
    _transportConfig.parentNodeId = loadState(101);
    _transportConfig.nodeId = myid;
    _transportConfig.distanceGW = loadState(103);
    mypar = _transportConfig.parentNodeId;
    nosleep = false;
    //flag_fcount = 1;
    err_delivery_beat = 6;
    happy_node_mode();
    gateway_fail();
  }

  if (nosleep == false) {
    oldmillis = millis();
    axelIntStatus = 0;
    buttIntStatus = 0;
    gerkIntStatus = 0;
#ifdef MAGNET_SENS
    magIntStatus = 0;
#endif
    sleep(SLEEP_TIME_W, false);
    nosleep = true;
  }
}


void blinky(uint8_t pulses, uint8_t repit, uint8_t ledColor) {
  for (int x = 0; x < repit; x++) {
    if (x > 0) {
      sleep(100);
    }
    for (int i = 0; i < pulses; i++) {
      if (i > 0) {
        sleep(40);
      }
      digitalWrite(ledColor, LOW);
      sleep(10);
      digitalWrite(ledColor, HIGH);
    }
  }
}


void board_Init() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
#ifdef MAGNET_SENS
  pinMode(MAGNET_INT, INPUT);
#endif
  pinMode(GERKON_INT, INPUT);
  pinMode(AXEL_INT, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  ledsOff();
  NRF_POWER->DCDCEN = 1;
  wait(5);
#ifndef MY_DEBUG
  NRF_UART0->ENABLE = 0;
  wait(5);
#endif
  NRF_NFCT->TASKS_DISABLE = 1;
  NRF_NVMC->CONFIG = 1;
  NRF_UICR->NFCPINS = 0;
  NRF_NVMC->CONFIG = 0;
  NRF_SAADC ->ENABLE = 0;
  NRF_PWM0  ->ENABLE = 0;
  NRF_PWM1  ->ENABLE = 0;
  NRF_PWM2  ->ENABLE = 0;
  NRF_TWIM1 ->ENABLE = 0;
  NRF_TWIS1 ->ENABLE = 0;
  //NRF_RADIO->TXPOWER = 8;
  wait(5);

  conf_vibro_set = loadState(230);
  if ((conf_vibro_set > 5) || (conf_vibro_set == 0)) {
    conf_vibro_set = 2;
    saveState(230, conf_vibro_set);
  }

  blinky(1, 1, GREEN_LED);
}


void ledsOff() {
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);
}


void happy_init() {
  //hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255); // ******************** checking the node config reset *************************

  if (hwReadConfig(EEPROM_NODE_ID_ADDRESS) == 0) {
    hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255);
  }
  if (loadState(100) == 0) {
    saveState(100, 255);
  }
  CORE_DEBUG(PSTR("EEPROM NODE ID: %d\n"), hwReadConfig(EEPROM_NODE_ID_ADDRESS));
  CORE_DEBUG(PSTR("USER MEMORY SECTOR NODE ID: %d\n"), loadState(100));

  if (hwReadConfig(EEPROM_NODE_ID_ADDRESS) == 255) {
    mtwr = 0;
  } else {
    mtwr = 6000;
    no_present();
  }
  CORE_DEBUG(PSTR("MY_TRANSPORT_WAIT_MS: %d\n"), mtwr);
}

void no_present() {
  _coreConfig.presentationSent = true;
  _coreConfig.nodeRegistered = true;
}


void interrupt_Init(bool start) {
  //***
  //NRF_GPIO_PIN_NOPULL
  //NRF_GPIO_PIN_PULLUP
  //NRF_GPIO_PIN_PULLDOWN
  //***
  nrf_gpio_cfg_input(PIN_BUTTON, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(AXEL_INT, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(GERKON_INT, NRF_GPIO_PIN_NOPULL);
#ifdef MAGNET_SENS
  nrf_gpio_cfg_input(MAGNET_INT, NRF_GPIO_PIN_NOPULL);
#endif
  APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
  PIN_BUTTON_MASK = 1 << PIN_BUTTON;
  AXEL_INT_MASK = 1 << AXEL_INT;
  GERKON_INT_MASK = 1 << GERKON_INT;
#ifdef MAGNET_SENS
  MAGNET_INT_MASK = 1 << MAGNET_INT;
#endif
  if (start == false) {

#ifdef MAGNET_SENS
    app_gpiote_user_register(&m_gpiote_user_id, AXEL_INT_MASK | GERKON_INT_MASK, GERKON_INT_MASK | MAGNET_INT_MASK | PIN_BUTTON_MASK, gpiote_event_handler);
#else
    app_gpiote_user_register(&m_gpiote_user_id, AXEL_INT_MASK | GERKON_INT_MASK, GERKON_INT_MASK | PIN_BUTTON_MASK, gpiote_event_handler);
#endif
    wait(5);
  } else if (start == true) {
#ifdef MAGNET_SENS
    app_gpiote_user_register(&m_gpiote_user_id, GERKON_INT_MASK, GERKON_INT_MASK | MAGNET_INT_MASK | PIN_BUTTON_MASK, gpiote_event_handler);
#else
    app_gpiote_user_register(&m_gpiote_user_id, GERKON_INT_MASK, GERKON_INT_MASK | PIN_BUTTON_MASK, gpiote_event_handler);
#endif
    wait(5);
  }
  app_gpiote_user_enable(m_gpiote_user_id);
  wait(5);
  axelIntStatus = 0;
  buttIntStatus = 0;
  gerkIntStatus = 0;
#ifdef MAGNET_SENS
  magIntStatus = 0;
#endif
}


void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
{
  MY_HW_RTC->CC[0] = (MY_HW_RTC->COUNTER + 2); // Taken from d0016 example code, ends the sleep delay

  if (PIN_BUTTON_MASK & event_pins_high_to_low) {
#ifdef MAGNET_SENS
    if ((buttIntStatus == 0) && (axelIntStatus == 0) && (gerkIntStatus == 0) && (magIntStatus == 0)) {
#else
    if ((buttIntStatus == 0) && (axelIntStatus == 0) && (gerkIntStatus == 0)) {
#endif
      buttIntStatus = PIN_BUTTON;
    }
  }
  if (flag_nogateway_mode == false) {
    if (AXEL_INT_MASK & event_pins_low_to_high) {
#ifdef MAGNET_SENS
      if ((axelIntStatus == 0) && (buttIntStatus == 0) && (gerkIntStatus == 0) && (magIntStatus == 0) && (door_status == 1)) {
#else
      if ((axelIntStatus == 0) && (buttIntStatus == 0) && (gerkIntStatus == 0) && (door_status == 1)) {
#endif
        axelIntStatus = AXEL_INT;
        axel_time0 = millis();
      }
    }
    if ((GERKON_INT_MASK & event_pins_low_to_high) || (GERKON_INT_MASK & event_pins_high_to_low)) {
#ifdef MAGNET_SENS
      if ((axelIntStatus == 0) && (buttIntStatus == 0) && (gerkIntStatus == 0) && (magIntStatus == 0)) {
#else
      if ((axelIntStatus == 0) && (buttIntStatus == 0) && (gerkIntStatus == 0)) {
#endif
        gerkIntStatus = GERKON_INT;
      }
    }
#ifdef MAGNET_SENS
    if (MAGNET_INT_MASK & event_pins_high_to_low) {
      if ((axelIntStatus == 0) && (buttIntStatus == 0) && (gerkIntStatus == 0) && (magIntStatus == 0) && (door_status == 1)) {
        magIntStatus = MAGNET_INT;
      }
    }
#endif
  }
}


void sensors_Init() {
  Wire.begin();
  wait(200);
  lis2 = new LIS2DW12Sensor (&Wire);
  vibro_Init();
  if (flag_nogateway_mode == false) {
    if (digitalRead(GERKON_INT) == HIGH) {
      door_status = true;
      interrupt_Init(false);
    } else {
      door_status = false;
      interrupt_Init(true);
    }
    send(dwsMsg.set(door_status));
    SLEEP_TIME_W = SLEEP_TIME;
    axelIntStatus = 0;
    buttIntStatus = 0;
    gerkIntStatus = 0;
#ifdef MAGNET_SENS
    magIntStatus = 0;
#endif
    sendBatteryStatus(false);
    blinky(2, 1, BLUE_LED);
    wait(100);
    blinky(2, 1, GREEN_LED);
    axel_time = millis();
  } else {
    interrupt_Init(true);
    blinky(5, 3, RED_LED);
  }
}


void config_Happy_node() {
  if (mtwr == 0) {
    myid = getNodeId();
    saveState(100, myid);
    mypar = _transportConfig.parentNodeId;
    old_mypar = mypar;
    saveState(101, mypar);
    saveState(102, _transportConfig.distanceGW);
  }
  if (mtwr != 0) {
    myid = getNodeId();
    if (myid != loadState(100)) {
      saveState(100, myid);
    }
    if (isTransportReady() == true) {
      mypar = _transportConfig.parentNodeId;
      if (mypar != loadState(101)) {
        saveState(101, mypar);
      }
      if (_transportConfig.distanceGW != loadState(102)) {
        saveState(102, _transportConfig.distanceGW);
      }
      present_only_parent();
    }
    if (isTransportReady() == false)
    {
      no_present();
      err_delivery_beat = 6;
      _transportConfig.nodeId = myid;
      _transportConfig.parentNodeId = loadState(101);
      _transportConfig.distanceGW = loadState(102);
      mypar = _transportConfig.parentNodeId;
      happy_node_mode();
      gateway_fail();
    }
  }
}


void send_Axel() {
  if (millis() - axel_time >= 5000) {
    blinky(1, 1, BLUE_LED);
    blinky(1, 1, RED_LED);
    blinky(1, 1, BLUE_LED);
    blinky(1, 1, RED_LED);

    send_a = send(vibroMsg.set(vibro));
    if (send_a == false) {
      wait(30);
      send_a = send(vibroMsg.set(vibro));
    }
    if (send_a == true) {
      err_delivery_beat = 0;
      if (flag_nogateway_mode == true) {
        flag_nogateway_mode = false;
        CORE_DEBUG(PSTR("MyS: NORMAL GATEWAY MODE\n"));
        err_delivery_beat = 0;
      }
    } else {
      _transportSM.failedUplinkTransmissions = 0;
      if (err_delivery_beat < 6) {
        err_delivery_beat++;
      }
      if (err_delivery_beat == 5) {
        if (flag_nogateway_mode == 0) {
          gateway_fail();
          CORE_DEBUG(PSTR("MyS: LOST GATEWAY MODE\n"));
        }
      }
    }
    axel_time = millis();
    axelIntStatus = 0;
    nosleep = false;
  } else {
    axelIntStatus = 0;
    nosleep = false;
  }
}


void send_Gerkon() {
  if (digitalRead(GERKON_INT) == HIGH) {
    door_status = true;
    interrupt_Init(false);
  } else {
    door_status = false;
    interrupt_Init(true);
  }
  if (door_status == true) {
    blinky(1, 1, GREEN_LED);
  } else {
    blinky(1, 1, RED_LED);
  }
  send_a = send(dwsMsg.set(door_status));
  if (send_a == false) {
    wait(20);
    send_a = send(dwsMsg.set(door_status));
    if (send_a == false) {
      wait(40);
      send_a = send(dwsMsg.set(door_status));
    }
  }
  if (send_a == true) {
    err_delivery_beat = 0;
    if (flag_nogateway_mode == true) {
      flag_nogateway_mode = false;
      CORE_DEBUG(PSTR("MyS: NORMAL GATEWAY MODE\n"));
      err_delivery_beat = 0;
    }
  } else {
    _transportSM.failedUplinkTransmissions = 0;
    if (err_delivery_beat < 6) {
      err_delivery_beat++;
    }
    if (err_delivery_beat == 5) {
      if (flag_nogateway_mode == false) {
        gateway_fail();
        CORE_DEBUG(PSTR("MyS: LOST GATEWAY MODE\n"));
      }
    }
  }
  gerkIntStatus = 0;
  nosleep = false;
}

#ifdef MAGNET_SENS
void send_Magnet() {
  blinky(2, 1, BLUE_LED);
  blinky(2, 1, GREEN_LED);
  blinky(2, 1, RED_LED);
  send_a = send(mMsg.set(magnet_status));
  if (send_a == false) {
    send_a = send(mMsg.set(magnet_status));
  }
  if (send_a == true) {
    err_delivery_beat = 0;
    if (flag_nogateway_mode == true) {
      flag_nogateway_mode = false;
      CORE_DEBUG(PSTR("MyS: NORMAL GATEWAY MODE\n"));
      err_delivery_beat = 0;
    }
  } else {
    _transportSM.failedUplinkTransmissions = 0;
    if (err_delivery_beat < 6) {
      err_delivery_beat++;
    }
    if (err_delivery_beat == 5) {
      if (flag_nogateway_mode == false) {
        gateway_fail();
        CORE_DEBUG(PSTR("MyS: LOST GATEWAY MODE\n"));
      }
    }
  }
  magIntStatus = 0;
  nosleep = false;
}
#endif

void new_device() {
  hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255);
  saveState(100, 255);
  hwReboot();
}


void update_Happy_transport() {
  CORE_DEBUG(PSTR("MyS: UPDATE TRANSPORT CONFIGURATION\n"));
  mypar = _transportConfig.parentNodeId;
  if (mypar != loadState(101))
  {
    saveState(101, mypar);
  }
  if (_transportConfig.distanceGW != loadState(102))
  {
    saveState(102, _transportConfig.distanceGW);
  }
  present_only_parent();
  wait(50);
  nosleep = false;
  flag_update_transport_param = false;
}


void present_only_parent() {
  if (old_mypar != mypar) {
    CORE_DEBUG(PSTR("MyS: SEND LITTLE PRESENT:) WITH PARENT ID\n"));
    if (_sendRoute(build(_msgTmp, 0, NODE_SENSOR_ID, C_INTERNAL, 6).set(mypar))) {
      flag_sendRoute_parent = false;
      old_mypar = mypar;
    } else {
      flag_sendRoute_parent = true;
    }
  }
}


void happy_node_mode() {
  _transportSM.findingParentNode = false;
  _transportSM.transportActive = true;
  _transportSM.uplinkOk = true;
  _transportSM.pingActive = false;
  _transportSM.failureCounter = 0u;
  _transportSM.uplinkOk = true;
  _transportSM.failureCounter = 0u;
  _transportSM.failedUplinkTransmissions = 0u;
  transportSwitchSM(stReady);
  CORE_DEBUG(PSTR("TRANSPORT: %d\n"), isTransportReady());
}


void gateway_fail() {
  flag_nogateway_mode = true;
  flag_update_transport_param = false;
  SLEEP_TIME_W = SLEEP_NOGW;
}


void check_parent() {
  _transportSM.findingParentNode = true;
  CORE_DEBUG(PSTR("MyS: SEND FIND PARENT REQUEST, WAIT RESPONSE\n"));
  _sendRoute(build(_msg, 255, NODE_SENSOR_ID, C_INTERNAL, 7).set(""));
  wait(600, C_INTERNAL, 8);
  if (_msg.sensor == 255) {
    if (mGetCommand(_msg) == C_INTERNAL) {
      if (_msg.type == 8) {
        Ack_FP = true;
        CORE_DEBUG(PSTR("MyS: PARENT RESPONSE FOUND\n"));
      }
    }
  }
  if (Ack_FP == true) {
    CORE_DEBUG(PSTR("MyS: FIND PARENT PROCESS\n"));
    Ack_FP = false;
    transportSwitchSM(stParent);
    flag_nogateway_mode = false;
    flag_find_parent_process = true;
    problem_mode_count = 0;
  } else {
    _transportSM.findingParentNode = false;
    CORE_DEBUG(PSTR("MyS: PARENT RESPONSE NOT FOUND\n"));
    _transportSM.failedUplinkTransmissions = 0;
    CORE_DEBUG(PSTR("TRANSPORT: %d\n"), isTransportReady());
    nosleep = false;
    if (problem_mode_count < 9) {
      CORE_DEBUG(PSTR("PROBLEM MODE COUNTER: %d\n"), problem_mode_count);
      problem_mode_count++;
      SLEEP_TIME_W = SLEEP_TIME_W + SLEEP_TIME_W;
    }
  }
}


void find_parent_process() {
  flag_update_transport_param = true;
  flag_find_parent_process = false;
  CORE_DEBUG(PSTR("MyS: STANDART TRANSPORT MODE IS RESTORED\n"));
  err_delivery_beat = 0;
  SLEEP_TIME_W = SLEEP_TIME;
  nosleep = false;
}


void sendBatteryStatus(bool start) {
  batt_cap = battery_level_in_percent(batteryVoltage);
  if (start == true) {
    if (batt_cap < old_batt_cap) {
      send_a = sendBatteryLevel(battery_level_in_percent(batteryVoltage), 1);
      //wait(1500, C_INTERNAL, I_BATTERY_LEVEL);
      if (send_a == false) {
        wait(30);
        send_a = sendBatteryLevel(battery_level_in_percent(batteryVoltage), 1);
      }
      old_batt_cap = batt_cap;
    }
  } else {
    send_a = sendBatteryLevel(battery_level_in_percent(batteryVoltage), 1);
    //wait(1500, C_INTERNAL, I_BATTERY_LEVEL);
    if (send_a == false) {
      wait(30);
      send_a = sendBatteryLevel(battery_level_in_percent(batteryVoltage), 1);
    }
  }
  linkQuality = calculationRxQuality();
  if (linkQuality != old_linkQuality) {
    wait(10);
    sendSignalStrength(linkQuality);
    old_linkQuality = linkQuality;
  }
}


bool sendSignalStrength(const int16_t level, const bool ack)
{
  return _sendRoute(build(_msgTmp, GATEWAY_ADDRESS, SIGNAL_Q_ID, C_SET, V_VAR1,
                          ack).set(level));
}
int16_t calculationRxQuality() {
  int16_t nRFRSSI_temp = transportGetReceivingRSSI();
  int16_t nRFRSSI = map(nRFRSSI_temp, -85, -40, 0, 100);
  if (nRFRSSI < 0) {
    nRFRSSI = 0;
  }
  if (nRFRSSI > 100) {
    nRFRSSI = 100;
  }
  return nRFRSSI;
}


void receive(const MyMessage & message)
{
  if (message.sensor == LEVEL_SENSIV_V_SENS_CHILD_ID) {
    if (message.type == V_VAR1) {
      conf_vibro_set = message.getByte();
      vibro_Init();
      saveState(230, conf_vibro_set);
      wait(150);
      send(conf_vsensMsg.set(conf_vibro_set));
      wait(150);
      blinky(3, 3, GREEN_LED);
      configMode = false;
      nosleep = false;
    }
  }
}


void vibro_Init() {
  if (conf_vibro_set == 1) {
    lis2->ODRTEMP = ODR_1Hz6_LP_ONLY;
  }
  if (conf_vibro_set == 2) {
    lis2->ODRTEMP = ODR_12Hz5;
  }
  if (conf_vibro_set == 3) {
    lis2->ODRTEMP = ODR_25Hz;
  }
  if (conf_vibro_set == 4) {
    lis2->ODRTEMP = ODR_100Hz;
  }
  if (conf_vibro_set == 5) {
    lis2->ODRTEMP = ODR_200Hz;
  }
  lis2->Enable_X();
  wait(100);
  lis2->Enable_Wake_Up_Detection();
  wait(100);
}
