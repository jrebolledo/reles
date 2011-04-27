#include <stdlib.h>
#include <avr/pgmspace.h>
#include "utils.h"
#include <avr/io.h>
#include <WProgram.h>
#include <Wire.h>
#include <DS1307.h>

extern unsigned long last_time_backup;
extern unsigned long last_time_relays_state_were_sent;
extern unsigned long last_time_datetime_synced; // if this variable is false trigger syndate packet every day
extern boolean sync_time_at_boot; // flag false when datetime has been adjusted at startup 
extern boolean check_control_on_boot;
extern unsigned long last_time_IO_change;
// buffer que contiene el parseo 
extern BufferRS232Packet bufferIn;
extern BufferRS232Packet bufferOut;

extern bufferMeas BufferMediciones[MAX_BUFFER_MEAS]; // cantidad de dispositivos buffereados en mediciones
extern IOcontrolDef IOCtrlDef[MAX_IOS]; // contiene la informacion de control actual de cada IO y sus referencias a la reglas de control
extern manualDef IOManualDef[MAX_IOS]; // una regla de control para cada IO
extern rulesDef IORulesDef[MAX_IOS]; // una regla diaria para cada IO

extern boolean data_restored; // if data has been restored don try it again
extern unsigned long last_rule_check;
extern int _io_0_pin;
extern int _io_1_pin;
extern int _io_2_pin;
extern int _io_3_pin;
extern int _io_4_pin;
extern int _io_5_pin;
extern int _io_6_pin;
extern int _io_7_pin;
extern int _io_8_pin;
extern int _io_9_pin;
extern int _io_10_pin;
extern int _io_11_pin;

void keepDatetimeUpdated() {
  if ((sync_time_at_boot & (millis() - last_time_datetime_synced > 10000)) | (millis() - last_time_datetime_synced > INTERVAL_DATETIME_SYNC)) {
    if (DEBUG) {
      Serial.println("\t(DatetimeSync)");
    }
    bufferOut.Method = RELE_UPDATE_DATETIME_METHOD;
    bufferOut.len_params = 1;
    bufferOut.params[0] = MY_DEFAULT_SLOT;
    bufferOut.Id = 66;
    sendPacket();
    last_time_datetime_synced = millis();
  }
}

void checkNewRS232Request() {
  readPacket(100); //Lee Uart y pone en la estructura el packete si este fue bien enviado
  if (bufferIn.isAvailable) {
    switch (bufferIn.Method) {
    case RELE_REGLA_MANUAL_METHOD: // regla manual
      if (DEBUG) {
        Serial.println("\t(REGLA MANUAL)");
      }
      printBuffer(false);
      processCtrlPacket(true);
      break;
    case RELE_REGLA_CONTROL_METHOD: // regla de control
      printBuffer(false);
      processCtrlPacket(false);
      break;
    case RELE_MEDICION_SENSORES_METHOD: // valor de sensores de un device solicitado
      printBuffer(false);
      storeMeasurements();
      break;
    case RELE_UPDATE_DATETIME_METHOD: // Nueva hora para actualizar reloj
      printBuffer(false);
      setDateTime();
      break;
      //processSyncRelayState();
    case RELE_CLEAN_EEPROM_METHOD:
      printBuffer(false);
      cleanEEPROM();
    default:
      break;
    }
  }
  // packet has been processed
  bufferIn.isAvailable = false;
}

void sendAck(){
  #if (DEBUG)
    Serial.println("\t(Ack)");
  #endif
  
  bufferOut.Method = bufferIn.Method;
  bufferOut.len_params = 1;
  bufferOut.params[0] = 0;
  bufferOut.Id = bufferIn.Id;
  sendPacket();
}

void processCtrlPacket(boolean type) { // true: manual, false: rule
  //rule> nIO + steps_num + IOs... + dev_meas + io_meas + delta_histeresis + pol_histeresis
  //     + end_step1 + ref1 + ... + end_stepN + refN
  //manual> nIO + IOs... + yon + mthon + don + hon + mon + yoff + mthoff + doff + hoff 
  //       + moff + u
  boolean rule_is_used;
  boolean notifyIOchanges = false;
  byte now[6];
  getDateTime(now); // store current date in local variable now
  // normalize date to minutes to compare with another date
  long now_norm = normDate(now);
  // clear manual rule assigned to IO target 
  #if (DEBUG)
    if (type) {
      Serial.print("\t(REGLA MANUAL) Para Ios:");
    }
    else {
      Serial.print("\t(REGLA UMBRAL) Para Ios:");
    }
  #endif
  
  for (int j = 0; j < bufferIn.params[0]; j++){
    if (type) {
      IOCtrlDef[bufferIn.params[j + 1]].manual_index = 0xF1;
      #if (DEBUG)
        Serial.print(bufferIn.params[j + 1],DEC);Serial.print(" ");
      #endif
    }
    else {
      IOCtrlDef[bufferIn.params[j + 2]].regla_index = 0xF1;
      #if (DEBUG) 
        Serial.print(bufferIn.params[j + 2],DEC);Serial.print(" ");
      #endif
    }
  }
  
  // search a rule which is not being used to put new control data on it

  int rule_index;
  for (rule_index = 0; rule_index < MAX_IOS; rule_index++) {
    rule_is_used = false;
    for (int io = 0; io < MAX_IOS; io++) {
      if (type) {
        if (IOCtrlDef[io].manual_index == rule_index) {
          rule_is_used = true;
          break;
        } 
      }
      else {
        if (IOCtrlDef[io].regla_index == rule_index) {
          rule_is_used = true;
          break;
        }
      }
    }
    if (!rule_is_used) { // save rule in available slot
      #if DEBUG
        Serial.print("\tSaved in:");Serial.println(rule_index,DEC);
      #endif
      
      if (type) {
        for (int j = 0; j < 5; j++){
          IOManualDef[rule_index].start_date[j] = bufferIn.params[j + 1 + bufferIn.params[0]];
          IOManualDef[rule_index].end_date[j] = bufferIn.params[j + 6 + bufferIn.params[0]];
        }
        IOManualDef[rule_index].state = bufferIn.params[bufferIn.len_params - 1];
      }
      else {
        IORulesDef[rule_index].steps_num = bufferIn.params[1];
        IORulesDef[rule_index].devid = bufferIn.params[2 + bufferIn.params[0]];
        IORulesDef[rule_index].signal_index = bufferIn.params[3 + bufferIn.params[0]];
        IORulesDef[rule_index].tol_histeresis = bufferIn.params[4 + bufferIn.params[0]];
        IORulesDef[rule_index].polaridad_histeresis = bufferIn.params[5 + bufferIn.params[0]];
        for (int j = 0; j < IORulesDef[rule_index].steps_num; j++){
          IORulesDef[rule_index].steps[j].step_end = bufferIn.params[2 * j + 6 + bufferIn.params[0]];
          IORulesDef[rule_index].steps[j].ref = bufferIn.params[2 * j + 7 + bufferIn.params[0]];
        }
        IORulesDef[rule_index].current_actuator = 0;
      }
      
      break;
    }
  }
    
  for (int i = 0; i < MAX_IOS; i++){ // attach rule to each target IO
    if (type) {
      if (IOCtrlDef[i].manual_index == 0xF1) {
        IOCtrlDef[i].manual_index = rule_index;
        printIOCtlrDef(i);
      }
    }
    else {
      if (IOCtrlDef[i].regla_index == 0xF1) {
        IOCtrlDef[i].regla_index = rule_index;
        printIOCtlrDef(i);
        
      }      
    }
  }

  printManualDef(rule_index);

  checkControlRules(true);
  
  // save changes in EEPROM
  backupData(false);
  sendAck();
}



void processSyncRelayState() { // if gateway ask for relay states
  sendRelaysState(bufferIn.Method,bufferIn.Id);
}

void keepRelayStateUpdatedonServer() { // executed every RELAY_STATE_REFRESH_INTERVAL

  if (last_time_relays_state_were_sent > millis()) {
    last_time_relays_state_were_sent = millis();
  }
  if (millis() - last_time_relays_state_were_sent > RELAY_STATE_REFRESH_INTERVAL) {
    #if (DEBUG)
      Serial.println("\t(AutoSync)");
    #endif
    
    sendRelaysState(RELE_MEDICON_REFRESH_METHOD,6);
    last_time_relays_state_were_sent = millis();
  }
}

void sendRelaysState(byte method, byte Id) { // funcion de bajo de nivel 
  // Estado Rele 1 On, 0 Off 
  bufferOut.len_params = MAX_IOS + 1;
  bufferOut.Method = method;
  bufferOut.params[0] = MY_DEFAULT_SLOT;
  for (int i = 0; i < MAX_IOS; i++){
    //IOCtrlDef[i].state = false;
    bufferOut.params[i+1] = IOCtrlDef[i].state;
  }
  bufferOut.Id = Id;
  sendPacket(); // pack and send bufferOut
  //backupData(); // debug to clean
}

void changeIOstate(byte IO_index, boolean change_to) {
  int IO_vector[MAX_IOS] = {
    _io_0_pin,_io_1_pin,_io_2_pin,_io_3_pin,_io_4_pin,_io_5_pin,_io_6_pin,_io_7_pin,_io_8_pin,_io_9_pin,_io_10_pin,_io_11_pin  };
  
  if (last_time_IO_change > millis()) {
    last_time_IO_change = millis();
  }
  
  while (true) {
    if (millis() - last_time_IO_change > MIN_INTERVAL_IO_CHANGE) {
      last_time_IO_change = millis();
      digitalWrite(IO_vector[IO_index], !change_to); //logica inversa
      break;
    }
  }
}

void checkControlRules(boolean forced) {
  if (last_rule_check > millis()) last_rule_check = millis();
  if (check_control_on_boot | forced | (millis() - last_rule_check > INTERVAL_RULE_CHECK)) {
    check_control_on_boot = false;
    last_rule_check = millis();
    #if (DEBUG)
      Serial.println("\t(CheckCtrl)");
    #endif
    // iterate over each IO and evaluate manual (is it applicable now?) or control rule,
    byte now[6];
    getDateTime(now); // store current date in local variable now
    #if (DEBUG)
      printDate(now);Serial.println();
    #endif
    // normalize date to minutes to compare with another date
    long now_norm = normDate(now);
    byte now_for_step_detection = now[3]*6 + now[4]/10; // 144 end of the day (00:00), now[4]/10 round to floor
    long start_date_norm, end_date_norm;
    // flag used to identify change that need to be notified to gateway
    boolean notifyIOchanges = false;
    boolean forced_state_for_control_rule=false; // as the control rule push its output to 0,1,>1, this variable store true, or false depending of the rule evaluation
  
    boolean meas_has_been_found  = true; // if measurement has been found
    byte meas_value; // store value measurement used to control IOs
    byte sup_hist; // limite superior de la banda de histeresis
    byte inf_hist; // limite inferior de la banda de histeresis
    byte cur_hist; // actual valor de salida de la histeresis
    boolean force_to_defult_state;
    boolean manual_isforcing_now;
    #if (DEBUG)
      Serial.print("\tNow:");Serial.print(now_norm,DEC);Serial.print("\tNow(step_ref)");Serial.println(now_for_step_detection,DEC);
    #endif

    for (int r = 0;r < MAX_IOS; r++) {
      force_to_defult_state = true;
      // evaluate manual rules
      #if (DEBUG)
        Serial.print("IO:");Serial.println(r,DEC);
      #endif
      
      manual_isforcing_now = false;
      if (IOCtrlDef[r].manual_index != 0xFF) { // evaluate only IO with valid manual rules
        force_to_defult_state = false;
        // normalize start datetime to minutes (maybe this data could be calculate once the rule has been saved, not every time the rule is ealuated (making this process faster)
        printManualDef(IOCtrlDef[r].manual_index);

        start_date_norm = normDate(IOManualDef[IOCtrlDef[r].manual_index].start_date);  
        end_date_norm = normDate(IOManualDef[IOCtrlDef[r].manual_index].end_date);
        
        if (now_norm >= start_date_norm && now_norm <= end_date_norm) { // compare date and check if manual rules applies now
          if (IOManualDef[IOCtrlDef[r].manual_index].state != IOCtrlDef[r].state) { // check if this manual control  modifies the current state of IO      
            #if (DEBUG)
              Serial.print("\tIO");
              Serial.print(r,DEC);
              Serial.print(" -> ");
              Serial.println(IOManualDef[IOCtrlDef[r].manual_index].state,DEC);
            #endif
            
            // change electrical state of IO to new state
            changeIOstate(r,IOManualDef[IOCtrlDef[r].manual_index].state);
            // update local state record
            IOCtrlDef[r].state = IOManualDef[IOCtrlDef[r].manual_index].state;
            // set flag to notify new changes in IO banks
            notifyIOchanges = true;
            
            
          }
          manual_isforcing_now = true;
        }
        else { // clean expired manual rule
          IOCtrlDef[r].manual_index = 0XFF;
        }
  
  
      }
      // evaluate control rules and
      if (IOCtrlDef[r].regla_index != 0xFF & !manual_isforcing_now) { // evaluate only IO with valid control rules
        force_to_defult_state = false;
        printReglaDef(IOCtrlDef[r].regla_index);
        // iterate over step and detect active step
        for (byte t = 0; t < IORulesDef[IOCtrlDef[r].regla_index].steps_num; t++) {
          if (now_for_step_detection < IORulesDef[IOCtrlDef[r].regla_index].steps[t].step_end) {
            #if (DEBUG)
              Serial.print("\tEvaluando step:");Serial.println(t,DEC);
            #endif
            // process rule, check if this step need to be avaluated with the threshold control procedure
            if (IORulesDef[IOCtrlDef[r].regla_index].steps[t].ref >= 2) { // 0: off, 1:on, > 1: threshold control
              // search measurement used by this rule
              #if (DEBUG)
                Serial.println("\tHisteresis");
              #endif
              
              meas_has_been_found = false;
    
              sup_hist = IORulesDef[IOCtrlDef[r].regla_index].steps[t].ref + IORulesDef[IOCtrlDef[r].regla_index].tol_histeresis/2;
              inf_hist = IORulesDef[IOCtrlDef[r].regla_index].steps[t].ref - IORulesDef[IOCtrlDef[r].regla_index].tol_histeresis/2;
              cur_hist = IOCtrlDef[r].state;
    
              for (byte k = 0; k < MAX_BUFFER_MEAS; k++) {
                if (BufferMediciones[k].devid == IORulesDef[IOCtrlDef[r].regla_index].devid) {
                  meas_has_been_found  = true;
                  meas_value = BufferMediciones[k].signals[IORulesDef[IOCtrlDef[r].regla_index].signal_index];
                  break;
                }
              }
    
              // compare with measurement buffer
              if (!meas_has_been_found) {
                #if (DEBUG)
                  Serial.println("\tMeas not found");
                #endif
                changeIOstate(r,IOCtrlDef[r].state);
                // update local state record
                break; // skip to next IO, this can not be evaluated becasute there is not a valid measurement in buffer
              }
              // process histeresis
              if (IORulesDef[IOCtrlDef[r].regla_index].polaridad_histeresis == 0) { // usado para control de luces (if pol:1 -> si medicion < ref -> action true,)
                if (cur_hist && (meas_value > sup_hist)) {
                  forced_state_for_control_rule =  false;
                }
                if (!cur_hist && (meas_value < inf_hist)) {
                  forced_state_for_control_rule =  true;
                }
                if (cur_hist && (meas_value < sup_hist)) {
                  forced_state_for_control_rule =  true;
                }
                if (!cur_hist && (meas_value > inf_hist)) {
                  forced_state_for_control_rule =  false;
                }
              }
              else { // usado para control de aire (if pol:1 -> si medicion < ref -> action false,)
                if (!cur_hist && (meas_value > sup_hist)) {
                  forced_state_for_control_rule =  true;
                }
                if (cur_hist && (meas_value < inf_hist)) {
                  forced_state_for_control_rule =  false;
                }
                if (!cur_hist && (meas_value < sup_hist)) {
                  forced_state_for_control_rule =  false;
                }
                if (cur_hist && (meas_value > inf_hist)) {
                  forced_state_for_control_rule =  true;
                }
              } // en process histeresis
            }
            else {  // forced control, timer
              #if (DEBUG)
                Serial.println("\tTimer");
              #endif
              if (IORulesDef[IOCtrlDef[r].regla_index].steps[t].ref == 0) {
                forced_state_for_control_rule =  false; // turn off
              }
              if (IORulesDef[IOCtrlDef[r].regla_index].steps[t].ref == 1) {
                forced_state_for_control_rule =  true; // turn on
              }
            } // en process timer control
    
            // check if this IO need to change its state
            if (forced_state_for_control_rule != IOCtrlDef[r].state) {
              #if (DEBUG)
                Serial.print("\tIO");
                Serial.print(r,DEC);
                Serial.print(" -> ");
                Serial.println(forced_state_for_control_rule,DEC);
              #endif
              if (forced_state_for_control_rule != false & forced_state_for_control_rule != true) {
                #if (DEBUG)
                  Serial.print("Error:");Serial.println(forced_state_for_control_rule,DEC);
                #endif
              }
              changeIOstate(r,forced_state_for_control_rule);
              // update local state record
              IOCtrlDef[r].state = forced_state_for_control_rule;
              // set flag to notify new changes in IO banks
              notifyIOchanges = true;
            } // update electrical state of each IO
            break;
          }
          
        }
        
        
      }
      
      if (force_to_defult_state) {
        changeIOstate(r,false);
        // update local state record
        IOCtrlDef[r].state = false;
        // set flag to notify new changes in IO banks
      }
      #if (DEBUG)
        Serial.print("\tIO");Serial.print(r,DEC);Serial.print(": ");Serial.println(int(IOCtrlDef[r].state),DEC);
      #endif
    }
  
    // if something has changed send new relay state to gateway, and sync relay state in EPROM
    if (notifyIOchanges) {   
      sendRelaysState(RELE_MEDICON_REFRESH_METHOD,45);
      backupData(false);
    }
  }


}

void storeMeasurements() {
  byte found = false;
  byte buffer_available_index = 255;
  // [DevID1 + Data1...6]
  ///for (int i = 0; i < bufferIn.params[0]; i++){
    //tempSpace = 0xff;
  for (int j = 0; j < MAX_BUFFER_MEAS; j++){
    if (BufferMediciones[j].devid == bufferIn.params[0]){
      for (int m = 0; m < MAX_SIGNALS_PER_DEV; m++){
        BufferMediciones[j].signals[m] = bufferIn.params[1+m];
      }
      found = true;
      #if (DEBUG)
        Serial.print("\t(MeasUpd):");
        printList(BufferMediciones[j].signals,0,MAX_SIGNALS_PER_DEV);
        Serial.println();
      #endif
      break;
    }
    else {
      if (BufferMediciones[j].devid == 0) {
        buffer_available_index = j;
      }
    }
  }
  
  if (!found & buffer_available_index != 255) {
    BufferMediciones[buffer_available_index].devid = bufferIn.params[0];
    for (int m = 0; m < MAX_SIGNALS_PER_DEV; m++){
      BufferMediciones[buffer_available_index].signals[m] = bufferIn.params[1+m];
    }
    #if (DEBUG)
      Serial.print("\t(MeasSv):");
      printList(BufferMediciones[buffer_available_index].signals,0,MAX_SIGNALS_PER_DEV);
      Serial.println();
    #endif
  }
  checkControlRules(true);
  
}

void setDateTime(){
  // Year, Month, Day, DayOfWeek, Hour, Minute
  #if (DEBUG)
    Serial.print("\t(SETTIME):");
  #endif
  
  RTC.stop();
  RTC.set(DS1307_SEC, (int)bufferIn.params[5]);        //set the seconds
  RTC.set(DS1307_MIN, (int)bufferIn.params[4]);     //set the minutes
  RTC.set(DS1307_HR, (int)bufferIn.params[3]);       //set the hours
  RTC.set(DS1307_DATE, (int)bufferIn.params[2]);       //set the date
  RTC.set(DS1307_MTH, (int)bufferIn.params[1]);        //set the month
  RTC.set(DS1307_YR, (int)bufferIn.params[0]);         //set the year
  RTC.start();
  sync_time_at_boot = false;
  #if DEBUG
    byte now[6];
    getDateTime(now);
    printDate(now);Serial.println();
  #endif
}

void getDateTime(byte* data) {
  data[0] = (byte)(RTC.get(DS1307_YR,true)-2000);
  data[1] = (byte)RTC.get(DS1307_MTH,true);
  data[2] = (byte)RTC.get(DS1307_DATE,true);
  data[3] = (byte)RTC.get(DS1307_HR,true);
  data[4] = (byte)RTC.get(DS1307_MIN,true);
  data[5] = (byte)RTC.get(DS1307_SEC,true);
}



////////////////////////////////////////////////////////////////
/////////////////// Comunicacion RS232 /////////////////////////
////////////////////////////////////////////////////////////////


// START_BYTE|Lenght|Metodo and data|
void readPacket(int timeToWait) {
  byte _checksumTotal = 0;
  byte _pos = 0;
  byte b = 0;
  boolean _escape = false; 
  // reset previous response
  if (bufferIn.isAvailable || (bufferIn.ErrorCode > 1)) {
    // Si nadie leyo el mensaje en un loop el mensaje es desechado
    bufferIn.isAvailable = false;
    bufferIn.ErrorCode = NO_ERROR;
  }
  if(Serial.available())delay(timeToWait);//Espera a que llegue todo el mensaje
  while (Serial.available()) {
    b = Serial.read();
    if (_pos == 0 && b != START_BYTE) {
      bufferIn.ErrorCode = UNEXPECTED_START_BYTE;
      return;
    }
    if (_pos > 0 && b == START_BYTE) {
      // new packet start before previous packeted completed -- discard previous packet and start over
      bufferIn.ErrorCode = UNEXPECTED_START_BYTE;
      _pos = 0;
      _checksumTotal = 0;
    }
    if (_pos > 0 && b == ESCAPE) {
      if (Serial.available()) {
        b = Serial.read();
        b = 0x20 ^ b;
      } 
      else {
        // escape byte.  next byte will be
        _escape = true;
        continue;
      }
    }
    if (_escape == true) {
      b = 0x20 ^ b;
      _escape = false;
    }
    // checksum includes all bytes after len
    if (_pos >= 2) {
      _checksumTotal+= b;
    }
    switch(_pos) {
    case 0:
      if (b == START_BYTE) {
        _pos++;
      }
      break;
    case 1:
      // length msb
      bufferIn.len_params = b;
      _pos++;
      break;
    case 2:
      bufferIn.Method = b;
      _pos++;
      break;
    default:
      // check if we're at the end of the packet
      // packet length does not include start, length, method or checksum bytes, so add 3
      if (_pos == (bufferIn.len_params + 4)) {
        // verify checksum
        if ((_checksumTotal & 0xff) == 0xff) {
          bufferIn.isAvailable = true;
          bufferIn.ErrorCode = NO_ERROR;
          bufferIn.Id = bufferIn.params[bufferIn.len_params];
        } 
        else {
          // checksum failed
          bufferIn.ErrorCode = CHECKSUM_FAILURE;
        }
        // reset state vars
        _pos = 0;
        _checksumTotal = 0;
        return;
      } 
      else {
        // add to params array, Sin start byte, lenght, method ni checksum
        bufferIn.params[_pos - 3] = b;
        _pos++;
      }
    }
  }
}

void sendPacket() {
  byte checksum = 0;
  printBuffer(true);
  sendByte(START_BYTE, false);
  // send length
  sendByte(bufferOut.len_params, true);
  // Metodo
  sendByte(bufferOut.Method, true);
  // compute checksum
  checksum+= bufferOut.Method;
  for (int i = 0; i < bufferOut.len_params; i++) {
    sendByte(bufferOut.params[i], true);
    checksum+= bufferOut.params[i];
  }
  sendByte(bufferOut.Id,true);
  checksum+= bufferOut.Id;
  
  checksum = 0xff - checksum;// perform 2s complement
  sendByte(checksum, true);// send checksum
  // send packet
}

void sendByte(byte b, boolean escape) {
  if (escape && (b == START_BYTE || b == ESCAPE || b == XON || b == XOFF)) {
    Serial.print(ESCAPE, BYTE);
    Serial.print(b ^ 0x20, BYTE);
  } 
  else Serial.print(b, BYTE);
}


void backupData(boolean isreset) {
  #if (DEBUG)
    Serial.println("\t(BKP)");
  #endif
  // save RAM control rules structures in EPROM even Measurements buffer
  int eprom_index = 0;
  // backup IOCtrlDef
  for (int t=0;t<MAX_IOS;t++) {
    if (isreset) {
      IOCtrlDef[t].manual_index = 0xFF;
      IOCtrlDef[t].regla_index = 0xFF;
      IOCtrlDef[t].state = false;
    }
    EEPROM_writeAnything(eprom_index, IOCtrlDef[t]);    
    eprom_index +=sizeof(IOCtrlDef[t]);
  }
  // backup IORulesDef
  for (int t=0;t<MAX_IOS;t++) {
    EEPROM_writeAnything(eprom_index, IORulesDef[t]);    
    eprom_index +=sizeof(IORulesDef[t]);
  }
  // backup IOManualDef
  for (int t=0;t<MAX_IOS;t++) {
    EEPROM_writeAnything(eprom_index, IOManualDef[t]);    
    eprom_index +=sizeof(IOManualDef[t]);
  }
  // backup BufferMediciones
  for (int t=0;t<MAX_BUFFER_MEAS;t++) {
    if (isreset) {
      BufferMediciones[t].devid = 0; // clean debug
    }
    EEPROM_writeAnything(eprom_index, BufferMediciones[t]);    
    eprom_index +=sizeof(BufferMediciones[t]);
  }  

  // store CRC http://www.linuxquestions.org/questions/programming-9/calculating-checksum-of-a-structure-275434/
  // check backup reading from eprom and comparing with stored CRC


}

void restoreData() {
  #if DEBUG
    Serial.println("\t(Restoring)");
  #endif
  int eprom_index = 0;

  // ask data to gateway (optional) to avoid using the EPROM all the time

//  // restore IOCtrlDef
  for (int t=0;t<MAX_IOS;t++) {
    EEPROM_readAnything(eprom_index, IOCtrlDef[t]);
    printIOCtlrDef(t);
    changeIOstate(t,IOCtrlDef[t].state);
    eprom_index +=sizeof(IOCtrlDef[t]);
  }
  // backup IORulesDef
  for (int t=0;t<MAX_IOS;t++) {
    EEPROM_readAnything(eprom_index, IORulesDef[t]);    
    eprom_index +=sizeof(IORulesDef[t]);
  }
  // backup IOManualDef
  for (int t=0;t<MAX_IOS;t++) {
    EEPROM_readAnything(eprom_index, IOManualDef[t]);    
    eprom_index +=sizeof(IOManualDef[t]);
  }
  // backup BufferMediciones
  for (int t=0;t<MAX_BUFFER_MEAS;t++) {
    EEPROM_readAnything(eprom_index, BufferMediciones[t]);    
    eprom_index +=sizeof(BufferMediciones[t]);
  } 
}

void printBuffer(boolean isoutbuffer) {
  if (DEBUG) {
    if (isoutbuffer) {
      Serial.print("\tMethod:");Serial.println(bufferOut.Method,DEC);
      Serial.print("\tParams:");
      printList(bufferOut.params,0,bufferOut.len_params);
      Serial.print("\tId:");Serial.println(bufferOut.Id,DEC);
    }
    else {
      Serial.print("\tMethod:");Serial.println(bufferIn.Method,DEC);
      Serial.print("\tParams:");
      printList(bufferIn.params,0,bufferIn.len_params);
      Serial.print("\n\tId:");Serial.println(bufferIn.Id,DEC);
    }
  }
}

void printIOCtlrDef(byte index) {
  if (DEBUG) {
    Serial.print("\tIO");
    Serial.println(index,DEC);
    Serial.print("\tM.index:");
    Serial.print(IOCtrlDef[index].manual_index,DEC);
    Serial.print("\tR.index:");
    Serial.print(IOCtrlDef[index].regla_index,DEC);
    Serial.print("\tState:");
    Serial.println(IOCtrlDef[index].state,DEC);
  }
}
void printManualDef(byte rule_index) {
  if (DEBUG) {
    Serial.print("\tIOManualDef:");
    Serial.println(rule_index,DEC);
    Serial.print("\tStart:");printDate(IOManualDef[rule_index].start_date);
    Serial.print("\tEnd:");printDate(IOManualDef[rule_index].end_date);
    Serial.print("\tstate:");Serial.println(IOManualDef[rule_index].state,DEC);
  }
}

void printReglaDef(byte rule_index) {
  if (DEBUG) {
    Serial.print("\tIORuleDef:");
    Serial.println(rule_index,DEC);
    Serial.print("\tSteps_num:");
    Serial.print(IORulesDef[rule_index].steps_num,DEC);
    Serial.print("\tdev:");
    Serial.print(IORulesDef[rule_index].devid,DEC);
    Serial.print("\tsignal_index:");
    Serial.println(IORulesDef[rule_index].signal_index,DEC);
    Serial.print("\ttol_hi:");
    Serial.print(IORulesDef[rule_index].tol_histeresis,DEC);
    Serial.print("\tpol_hi:");
    Serial.println(IORulesDef[rule_index].polaridad_histeresis,DEC);
    
    for (int j = 0; j < IORulesDef[rule_index].steps_num; j++){
      Serial.print("\tEnd:");
      Serial.print(IORulesDef[rule_index].steps[j].step_end,DEC);
      Serial.print("\tref:");
      Serial.println(IORulesDef[rule_index].steps[j].ref,DEC);
    }
  }
}

void printList(byte *cadena, byte start_index, byte len) {
  #if DEBUG
    for (int y=start_index;y<start_index+len-1;y++) {
      Serial.print(cadena[y],DEC);Serial.print(", ");
    }
    Serial.print(cadena[start_index + len-1],DEC);
  #endif
}


void printDate(byte * cadena) {
  #if DEBUG
    printList(cadena,0,6);
  #endif
}


long normDate(byte * cadena) {
  return (long)(cadena[0]*365*30*24*60 + cadena[1]*30*24*60 + cadena[2]*24*60 + cadena[3]*60 + cadena[4]);
}

void cleanEEPROM() {
  backupData(true);
}



