#include <avr/pgmspace.h>
#include <avr/io.h>
#include "utils.h"
#include <WProgram.h>
#include <Wire.h>
#include <DS1307.h>
#include <EEPROM.h>

unsigned long last_time_backup=0; // last time backup was performed
unsigned long last_time_relays_state_were_sent = 0;// last time relay state were pushed to gateway 
unsigned long last_time_datetime_synced = 0;
unsigned long last_time_IO_change = 0;
boolean sync_time_at_boot = true;
boolean check_control_on_boot = true;
int _io_0_pin = io_0_PIN;
int _io_1_pin = io_1_PIN;
int _io_2_pin = io_2_PIN;
int _io_3_pin = io_3_PIN;
int _io_4_pin = io_4_PIN;
int _io_5_pin = io_5_PIN;
int _io_6_pin = io_6_PIN;
int _io_7_pin = io_7_PIN;
int _io_8_pin = io_8_PIN;
int _io_9_pin = io_9_PIN;
int _io_10_pin = io_10_PIN;
int _io_11_pin = io_11_PIN;

bufferMeas BufferMediciones[MAX_BUFFER_MEAS]; // cantidad de dispositivos buffereados en mediciones

IOcontrolDef IOCtrlDef[MAX_IOS]; // contiene la informacion de control actual de cada IO y sus referencias a la reglas de control

manualDef IOManualDef[MAX_IOS]; // una regla de control para cada IO

rulesDef IORulesDef[MAX_IOS]; // una regla diaria para cada IO

BufferRS232Packet bufferIn;
BufferRS232Packet bufferOut;

unsigned long last_rule_check = 0;

void setup() {
  Serial.begin(9600);  // UART
  pinMode(_io_0_pin,OUTPUT);
  pinMode(_io_1_pin,OUTPUT);
  pinMode(_io_2_pin,OUTPUT);
  pinMode(_io_3_pin,OUTPUT);
  pinMode(_io_4_pin,OUTPUT);
  pinMode(_io_5_pin,OUTPUT);
  pinMode(_io_6_pin,OUTPUT);
  pinMode(_io_7_pin,OUTPUT);
  pinMode(_io_8_pin,OUTPUT);
  pinMode(_io_9_pin,OUTPUT);
  pinMode(_io_10_pin,OUTPUT);
  pinMode(_io_11_pin,OUTPUT);
  #if (DEBUG)
    Serial.print("\t(Starting) :(");
  #endif
  
  #if (DEBUG)
    byte now[6];
    getDateTime(now);
    printDate(now);Serial.println(")");
  #endif
  //backupData(true);
  restoreData(); // restore rule data from gateway if not available load data from local EPROM

}

void loop() {
  //backupData(); // save all structure data ones a day

  keepDatetimeUpdated();
  
  checkNewRS232Request();
  
  checkControlRules(false);
  
  keepRelayStateUpdatedonServer(); // periodically, in a way to mitigate gateway shutdown or lack of connectivity with server
}



