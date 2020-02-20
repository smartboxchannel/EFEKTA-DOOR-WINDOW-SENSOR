extern "C" {
#include "app_gpiote.h"
#include "nrf_gpio.h"
}
#define APP_GPIOTE_MAX_USERS 1
#include <LIS2DW12Sensor.h>
//#define MY_DEBUG
#ifndef MY_DEBUG
#define MY_DISABLED_SERIAL
#endif
#define MY_RADIO_NRF5_ESB
int16_t mtwr;
#define MY_TRANSPORT_WAIT_READY_MS (mtwr)
#define MY_NRF5_ESB_PA_LEVEL (NRF5_PA_MAX)

#include <MySensors.h>

#define DWS_CHILD_ID 0
#define V_SENS_CHILD_ID 1
#define M_CHILD_ID 2
#define SIGNAL_Q_ID 250

MyMessage dwsMsg(DWS_CHILD_ID, V_TRIPPED);
MyMessage mMsg(M_CHILD_ID, V_TRIPPED);
MyMessage vibroMsg(V_SENS_CHILD_ID, V_TRIPPED);
#define SN "DOOR & WINDOW SENS"
#define SV "1.12"

int8_t int_status = 0;
bool door_status;
bool check;
bool magnet_status = 1;
bool nosleep = 0;
bool button_flag = 0;
bool onoff = 1;
bool flag_update_transport_param;
bool flag_sendRoute_parent;
bool flag_no_present;
bool flag_nogateway_mode;
bool flag_find_parent_process;
bool flag_fcount;
bool Ack_TL;
bool Ack_FP;
bool PRESENT_ACK;
bool send_a;
bool batt_flag;
byte err_delivery_beat;
byte problem_mode_count;
uint8_t  countbatt = 0;
uint8_t batt_cap;
uint8_t old_batt_cap = 100;
uint32_t BATT_TIME;
uint32_t SLEEP_TIME = 10800000;
uint32_t SLEEP_NOGW = 60000;
uint32_t oldmillis;
uint32_t newmillis;
uint32_t previousMillis;
uint32_t lightMillisR;
uint32_t interrupt_time;
uint32_t SLEEP_TIME_W;
uint32_t axel_time;
uint32_t axel_time0;
int16_t myid;
int16_t mypar;
int16_t old_mypar = -1;
bool vibro = 1;
static app_gpiote_user_id_t m_gpiote_user_id;
uint32_t PIN_BUTTON_MASK;
uint32_t AXEL_INT_MASK;
uint32_t GERKON_INT_MASK;
uint32_t MAGNET_INT_MASK;
float ODR_1Hz6_LP_ONLY = 1.6f;
float ODR_12Hz5 = 12.5f;
float ODR_25Hz = 25.0f;
float ODR_50Hz = 50.0f;
float ODR_100Hz = 100.0f;
float ODR_200Hz = 200.0f;
volatile byte axelIntStatus = 0;
volatile byte gerkIntStatus = 0;
volatile byte magIntStatus = 0;
volatile byte buttIntStatus = 0;
uint16_t batteryVoltage;
int16_t linkQuality;
int16_t old_linkQuality;
LIS2DW12Sensor *lis2;


void before() {
  board_Init();
  happy_init();
  delay(500);
  batteryVoltage = hwCPUVoltage();
  digitalWrite(BLUE_LED, LOW);
}


void presentation()
{
  NRF_POWER->DCDCEN = 0;
  wait(10);

  check = sendSketchInfo(SN, SV);
  wait(30);
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(30);
    check = sendSketchInfo(SN, SV);
    wait(60);
    _transportSM.failedUplinkTransmissions = 0;
  }
  if (check) {
    blinky(1, 1, BLUE_LED);
  } else {
    blinky(1, 1, RED_LED);
  }

  check = present(DWS_CHILD_ID, S_DOOR, "STATUS RS SENS");
  wait(40);
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(40);
    check = present(DWS_CHILD_ID, S_DOOR, "STATUS RS SENS");
    wait(80);
    _transportSM.failedUplinkTransmissions = 0;
  }
  if (check) {
    blinky(1, 1, BLUE_LED);
  } else {
    blinky(1, 1, RED_LED);
  }

  check = present(V_SENS_CHILD_ID, S_VIBRATION, "STATUS SHOCK SENS");
  wait(50);
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(50);
    check = present(V_SENS_CHILD_ID, S_VIBRATION, "STATUS SHOCK SENS");
    wait(100);
    _transportSM.failedUplinkTransmissions = 0;
  }
  if (check) {
    blinky(1, 1, BLUE_LED);
  } else {
    blinky(1, 1, RED_LED);
  }

  check = present(M_CHILD_ID, S_DOOR, "ANTI-MAGNET ALARM");
  wait(60);
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(60);
    check = present(M_CHILD_ID, S_DOOR, "ANTI-MAGNET ALARM");
    wait(120);
    _transportSM.failedUplinkTransmissions = 0;
  }
  if (check) {
    blinky(1, 1, BLUE_LED);
  } else {
    blinky(1, 1, RED_LED);
  }

  check = present(SIGNAL_Q_ID, S_CUSTOM, "SIGNAL %");
  wait(70);
  if (!check) {
    _transportSM.failedUplinkTransmissions = 0;
    wait(70);
    check = present(SIGNAL_Q_ID, S_CUSTOM, "SIGNAL %");
    wait(140);
    _transportSM.failedUplinkTransmissions = 0;
  }
  if (check) {
    blinky(1, 1, BLUE_LED);
  } else {
    blinky(1, 1, RED_LED);
  }
  wait(200);
  sendBatteryStatus(0);
  NRF_POWER->DCDCEN = 0;
  wait(10);
}


void setup() {
  digitalWrite(BLUE_LED, HIGH);
  config_Happy_node();
  sensors_Init();
}


void loop() {
  if (flag_update_transport_param == 1) {
    update_Happy_transport();
  }
  if (flag_sendRoute_parent == 1) {
    present_only_parent();
  }
  if (isTransportReady() == true) {
    if (flag_nogateway_mode == 0) {
      if (flag_find_parent_process == 1) {
        find_parent_process();
      }
      if ((axelIntStatus == AXEL_INT) || (buttIntStatus == PIN_BUTTON) || (gerkIntStatus == GERKON_INT) || (magIntStatus == MAGNET_INT)) {
        nosleep = 1;
        newmillis = millis();
        interrupt_time = newmillis - oldmillis;
        BATT_TIME = BATT_TIME - interrupt_time;
        if (BATT_TIME < 60000) {
          BATT_TIME = SLEEP_TIME;
          batteryVoltage = hwCPUVoltage();
          batt_flag = 1;
        }

        if (gerkIntStatus == GERKON_INT) {
          send_Gerkon();
          axel_time = millis();
          nosleep = 0;
        }

        if (magIntStatus == MAGNET_INT) {
          send_Magnet();
          nosleep = 0;
        }

        if (axelIntStatus == AXEL_INT) {
          if (millis() - axel_time0 >= 2000) {
            send_Axel();
            nosleep = 0;
          } else {
            if (digitalRead(GERKON_INT) == LOW) {
              send_Gerkon();
              axel_time = millis();
              nosleep = 0;
            }
          }
        }

        if (buttIntStatus == PIN_BUTTON) {
          if (digitalRead(PIN_BUTTON) == 0 && button_flag == 0) {
            button_flag = 1;
            previousMillis = millis();
            ledsOff();
          }
          if (digitalRead(PIN_BUTTON) == 0 && button_flag == 1) {
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

          if (digitalRead(PIN_BUTTON) == 1 && button_flag == 1) {
            if ((millis() - previousMillis <= 1750) && (button_flag == 1))
            {
              ledsOff();
              blinky(2, 2, BLUE_LED);
              button_flag = 0;
              buttIntStatus = 0;
              presentation();
              nosleep = 0;
            }
            if ((millis() - previousMillis > 2000) && (millis() - previousMillis <= 4000) && (button_flag == 1))
            {
              ledsOff();
              blinky(3, 3, RED_LED);
              //new_device();
            }

            if ((((millis() - previousMillis > 1750) && (millis() - previousMillis <= 2000)) || ((millis() - previousMillis > 4000))) && (button_flag == 1))
            {
              ledsOff();
              nosleep = 0;
              button_flag = 0;
              buttIntStatus = 0;
            }
          }
        }
      } else {
        batteryVoltage = hwCPUVoltage();
        BATT_TIME = SLEEP_TIME;
        sendBatteryStatus(1);
        nosleep = 0;
      }
    } else {
      if (buttIntStatus == PIN_BUTTON) {
        if (digitalRead(PIN_BUTTON) == 0 && button_flag == 0) {
          button_flag = 1;
          nosleep = 1;
          previousMillis = millis();
          ledsOff();
        }
        if (digitalRead(PIN_BUTTON) == 0 && button_flag == 1) {
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

        if (digitalRead(PIN_BUTTON) == 1 && button_flag == 1) {
          if ((millis() - previousMillis <= 1750) && (button_flag == 1))
          {
            ledsOff();
            blinky(2, 2, BLUE_LED);
            button_flag = 0;
            buttIntStatus = 0;
            check_parent();
            nosleep = 0;
          }
          if ((millis() - previousMillis > 2000) && (millis() - previousMillis <= 4000) && (button_flag == 1))
          {
            ledsOff();
            blinky(3, 3, RED_LED);
            //new_device();
          }

          if ((((millis() - previousMillis > 1750) && (millis() - previousMillis <= 2000)) || ((millis() - previousMillis > 4000))) && (button_flag == 1))
          {
            ledsOff();
            nosleep = 0;
            button_flag = 0;
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
    nosleep = 0;
    flag_fcount = 1;
    err_delivery_beat = 6;
    happy_node_mode();
    gateway_fail();
  }

  if (nosleep == 0) {
    oldmillis = millis();
    axelIntStatus = 0;
    buttIntStatus = 0;
    gerkIntStatus = 0;
    magIntStatus = 0;
    sleep(SLEEP_TIME_W, false);
    nosleep = 1;
  }
}


void blinky(uint8_t pulses, uint8_t repit, uint8_t ledColor) {
  for (int x = 0; x < repit; x++) {
    if (x > 0) {
      wait(150);
    }
    for (int i = 0; i < pulses; i++) {
      if (i > 0) {
        wait(40);
      }
      digitalWrite(ledColor, LOW);
      wait(10);
      digitalWrite(ledColor, HIGH);
    }
  }
}


void board_Init() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(MAGNET_INT, INPUT);
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
  NRF_RADIO->TXPOWER = 8;
  wait(5);
  blinky(1, 1, BLUE_LED);
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
    mtwr = 11000;
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
  //SET
  //NRF_GPIO_PIN_NOPULL
  //NRF_GPIO_PIN_PULLUP
  //NRF_GPIO_PIN_PULLDOWN
  //***
  nrf_gpio_cfg_input(PIN_BUTTON, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(AXEL_INT, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(GERKON_INT, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(MAGNET_INT, NRF_GPIO_PIN_NOPULL);
  APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
  PIN_BUTTON_MASK = 1 << PIN_BUTTON;
  AXEL_INT_MASK = 1 << AXEL_INT;
  GERKON_INT_MASK = 1 << GERKON_INT;
  MAGNET_INT_MASK = 1 << MAGNET_INT;
  //  app_gpiote_user_register(p_user_id, pins_low_to_high_mask, pins_high_to_low_mask, event_handler)
  if (start == 0) {
    app_gpiote_user_register(&m_gpiote_user_id, AXEL_INT_MASK | GERKON_INT_MASK, GERKON_INT_MASK | MAGNET_INT_MASK | PIN_BUTTON_MASK, gpiote_event_handler);
    wait(5);
  } else if (start == 1) {
    app_gpiote_user_register(&m_gpiote_user_id, GERKON_INT_MASK, GERKON_INT_MASK | MAGNET_INT_MASK | PIN_BUTTON_MASK, gpiote_event_handler);
    wait(5);
  }
  app_gpiote_user_enable(m_gpiote_user_id);
  wait(5);
  axelIntStatus = 0;
  buttIntStatus = 0;
  gerkIntStatus = 0;
  magIntStatus = 0;
}


void gpiote_event_handler(uint32_t event_pins_low_to_high, uint32_t event_pins_high_to_low)
{
  MY_HW_RTC->CC[0] = (MY_HW_RTC->COUNTER + 2); // Taken from d0016 example code, ends the sleep delay

  if (PIN_BUTTON_MASK & event_pins_high_to_low) {
    if ((buttIntStatus == 0) && (axelIntStatus == 0) && (gerkIntStatus == 0) && (magIntStatus == 0)) {
      buttIntStatus = PIN_BUTTON;
    }
  }
  if (flag_nogateway_mode == 0) {
    if (AXEL_INT_MASK & event_pins_low_to_high) {
      if ((axelIntStatus == 0) && (buttIntStatus == 0) && (gerkIntStatus == 0) && (magIntStatus == 0) && (door_status == 0)) {
        axelIntStatus = AXEL_INT;
        axel_time0 = millis();
      }
    }
    if ((GERKON_INT_MASK & event_pins_low_to_high) || (GERKON_INT_MASK & event_pins_high_to_low)) {
      if ((axelIntStatus == 0) && (buttIntStatus == 0) && (gerkIntStatus == 0) && (magIntStatus == 0)) {
        gerkIntStatus = GERKON_INT;
      }
    }
    if (MAGNET_INT_MASK & event_pins_high_to_low) {
      if ((axelIntStatus == 0) && (buttIntStatus == 0) && (gerkIntStatus == 0) && (magIntStatus == 0) && (door_status == 0)) {
        magIntStatus = MAGNET_INT;
      }
    }
  }
}


void sensors_Init() {
  Wire.begin();
  wait(100);
  lis2 = new LIS2DW12Sensor (&Wire);
  lis2->ODRTEMP = ODR_12Hz5;  //ODR_1Hz6_LP_ONLY;
  lis2->Enable_X();
  wait(50);
  lis2->Enable_Wake_Up_Detection();
  wait(50);
  if (flag_nogateway_mode == 0) {
    if (digitalRead(GERKON_INT) == HIGH) {
      door_status = 0;
      interrupt_Init(0);
    } else {
      door_status = 1;
      interrupt_Init(1);
    }
    send(dwsMsg.set(door_status));
    wait(50);

    SLEEP_TIME_W = SLEEP_TIME;
    axelIntStatus = 0;
    buttIntStatus = 0;
    gerkIntStatus = 0;
    magIntStatus = 0;
    sendBatteryStatus(0);
    wait(100);
     blinky(2, 1, BLUE_LED);
    wait(100);
    blinky(2, 1, GREEN_LED);
    wait(100);
    blinky(2, 1, RED_LED);
    axel_time = millis();
  } else {
    interrupt_Init(0);
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
      flag_fcount = 1;
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
    blinky(2, 1, GREEN_LED);
    blinky(2, 1, RED_LED);
    blinky(2, 1, GREEN_LED);
    blinky(2, 1, RED_LED);
    blinky(2, 1, GREEN_LED);
    blinky(2, 1, RED_LED);

    send_a = send(vibroMsg.set(vibro));
    wait(50);
    if (send_a == false) {
      send_a = send(vibroMsg.set(vibro));
      wait(100);
    }
    if (send_a == true) {
      err_delivery_beat = 0;
      if (flag_nogateway_mode == 1) {
        flag_nogateway_mode = 0;
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
    nosleep = 0;
  } else {
    axelIntStatus = 0;
    nosleep = 0;
  }
}


void send_Gerkon() {
  if (digitalRead(GERKON_INT) == HIGH) {
    door_status = 0;
    interrupt_Init(0);
  } else {
    door_status = 1;
    interrupt_Init(1);
  }
  if (door_status == 0) {
    blinky(1, 1, GREEN_LED);
  } else {
    blinky(1, 1, RED_LED);
  }
  send_a = send(dwsMsg.set(door_status));
  wait(50);
  if (send_a == false) {
    send_a = send(dwsMsg.set(door_status));
    wait(100);
    if (send_a == false) {
      send_a = send(dwsMsg.set(door_status));
      wait(150);
    }
  }
  if (send_a == true) {
    err_delivery_beat = 0;
    if (flag_nogateway_mode == 1) {
      flag_nogateway_mode = 0;
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
  gerkIntStatus = 0;
  nosleep = 0;
}


void send_Magnet() {
  blinky(2, 1, BLUE_LED);
  blinky(2, 1, RED_LED);
  blinky(2, 1, BLUE_LED);
  blinky(2, 1, RED_LED);
  blinky(2, 1, BLUE_LED);
  blinky(2, 1, RED_LED);
  send_a = send(mMsg.set(magnet_status));
  wait(50);
  if (send_a == false) {
    send_a = send(mMsg.set(magnet_status));
    wait(100);
  }
  if (send_a == true) {
    err_delivery_beat = 0;
    if (flag_nogateway_mode == 1) {
      flag_nogateway_mode = 0;
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
  magIntStatus = 0;
  nosleep = 0;
}


void new_device() {
  hwWriteConfig(EEPROM_NODE_ID_ADDRESS, 255);
  saveState(100, 255);
  wdt_enable(WDTO_15MS);
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
  nosleep = 0;
  flag_update_transport_param = 0;
}


void present_only_parent() {
  if (old_mypar != mypar) {
    CORE_DEBUG(PSTR("MyS: SEND LITTLE PRESENT:) WITH PARENT ID\n"));
    if (_sendRoute(build(_msgTmp, 0, NODE_SENSOR_ID, C_INTERNAL, 6).set(mypar))) {
      flag_sendRoute_parent = 0;
      old_mypar = mypar;
    } else {
      flag_sendRoute_parent = 1;
    }
  }
}


void happy_node_mode() {
  _transportSM.findingParentNode = false;
  _transportSM.transportActive = true;
  _transportSM.uplinkOk = true;
  _transportSM.pingActive = false;
  _transportSM.failureCounter = 0;
  _transportSM.uplinkOk = true;
  _transportSM.failureCounter = 0u;
  _transportSM.failedUplinkTransmissions = 0u;
  transportSwitchSM(stReady);
  CORE_DEBUG(PSTR("TRANSPORT: %d\n"), isTransportReady());
}


void gateway_fail() {
  flag_nogateway_mode = 1;
  flag_update_transport_param = 0;
  SLEEP_TIME_W = SLEEP_NOGW;
}


void check_parent() {
  _transportSM.findingParentNode = true;
  CORE_DEBUG(PSTR("MyS: SEND FIND PARENT REQUEST, WAIT RESPONSE\n"));
  _sendRoute(build(_msg, 255, NODE_SENSOR_ID, C_INTERNAL, 7).set(""));
  wait(1500, C_INTERNAL, 8);
  if (_msg.sensor == 255) {
    if (mGetCommand(_msg) == 3) {
      if (_msg.type == 8) {
        Ack_FP = 1;
        CORE_DEBUG(PSTR("MyS: PARENT RESPONSE FOUND\n"));
      }
    }
  }
  if (Ack_FP == 1) {
    CORE_DEBUG(PSTR("MyS: FIND PARENT PROCESS\n"));
    Ack_FP = 0;
    transportSwitchSM(stParent);
    flag_nogateway_mode = 0;
    flag_find_parent_process = 1;
    problem_mode_count = 0;
  } else {
    _transportSM.findingParentNode = false;
    CORE_DEBUG(PSTR("MyS: PARENT RESPONSE NOT FOUND\n"));
    _transportSM.failedUplinkTransmissions = 0;
    CORE_DEBUG(PSTR("TRANSPORT: %d\n"), isTransportReady());
    nosleep = 0;
    if (problem_mode_count < 9) {
      CORE_DEBUG(PSTR("PROBLEM MODE COUNTER: %d\n"), problem_mode_count);
      problem_mode_count++;
      SLEEP_TIME_W = SLEEP_TIME_W + SLEEP_TIME_W;
    }
  }
}


void find_parent_process() {
  flag_update_transport_param = 1;
  flag_find_parent_process = 0;
  CORE_DEBUG(PSTR("MyS: STANDART TRANSPORT MODE IS RESTORED\n"));
  err_delivery_beat = 0;
  SLEEP_TIME_W = SLEEP_TIME;
  nosleep = 0;
}


void sendBatteryStatus(bool start) {
  batt_cap = battery_level_in_percent(batteryVoltage);
  if (start == 1) {
    if (batt_cap < old_batt_cap) {
      sendBatteryLevel(battery_level_in_percent(batteryVoltage), 1);
      wait(1500, C_INTERNAL, I_BATTERY_LEVEL);
      old_batt_cap = batt_cap;
    }
  } else {
    sendBatteryLevel(battery_level_in_percent(batteryVoltage), 1);
    wait(1500, C_INTERNAL, I_BATTERY_LEVEL);
  }

  linkQuality = calculationRxQuality();
  if (linkQuality != old_linkQuality) {
    wait(10);
    sendSignalStrength(linkQuality);
    wait(50);
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
