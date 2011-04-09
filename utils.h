#ifndef utils_h
#define utils_h
#include <WProgram.h>
#include <Wire.h>
#include <DS1307.h>
#include <EEPROM.h>

#define DEBUG false

// METHODS
#define RELE_REGLA_MANUAL_METHOD 23
#define RELE_REGLA_CONTROL_METHOD 24
#define RELE_MEDICON_REFRESH_METHOD 25 // method envio de relays state
#define RELE_MEDICION_SENSORES_METHOD 26 
#define RELE_UPDATE_DATETIME_METHOD 27
#define RELE_CLEAN_EEPROM_METHOD 28 // reset eeprom

#define MY_DEFAULT_SLOT 2

#define PARSER_RS232_PARAMS_MAX 30 // maximo numero de parametros que se pueden almacenar por I2C request
#define MAX_SIGNALS_PER_DEV 6 // señales almacenadas por dev  (nodo con antena que mida)
#define MAX_STEPS_PER_RULE 4 // maxima cantidad de steps por regla
#define MAX_BUFFER_MEAS 3 // cantidad de nodos que son buffereados para mediciones
#define MAX_IOS 12 // cantidad de IOS controlados por una regla manual y por una regla de control

#define START_BYTE 0x7e
#define ESCAPE 0x7d
#define XON 0x11
#define XOFF 0x13

#define NO_ERROR 0
#define UNEXPECTED_START_BYTE 1
#define CHECKSUM_FAILURE 2
#define PACKET_EXCEEDS_BYTE_ARRAY_LENGTH 3

// Lighting 1,2,3,4
#define io_0_PIN 2
#define io_1_PIN 3
#define io_2_PIN 4
#define io_3_PIN 5

#define io_4_PIN 10
#define io_5_PIN 11
// HVAC 1,2,3,4
#define io_6_PIN 6
#define io_7_PIN 7
#define io_8_PIN 8
#define io_9_PIN 9

// not used
#define io_10_PIN 12
#define io_11_PIN 13




#define BACKUP_INTERVAL 86400000 // once a day
#define RESTORE_GATEWAY_TIMEOUT 20000 // 10 seg maximum
#define RELAY_STATE_REFRESH_INTERVAL 60000 // 1 minutes
#define INTERVAL_RULE_CHECK 15000
#define INTERVAL_DATETIME_SYNC 86400000 // once a day
#define MIN_INTERVAL_IO_CHANGE 300
/* STRUCTURES */

typedef struct BufferRS232Packet_struct {
  boolean isAvailable;
  byte ErrorCode;
  byte Method;
  byte params[PARSER_RS232_PARAMS_MAX];
  byte len_params;
  byte Id;
}BufferRS232Packet;

typedef struct bufferMeas_struct {
  byte devid; // dev ID
  byte signals[MAX_SIGNALS_PER_DEV]; // señal 
}bufferMeas;

typedef struct IOcontrolDef_struct {
  boolean type; // tipo de regla actual vigente (manual:true, regla:false)
  byte manual_index; // index de donde se encuentra la regla de control manual
  byte regla_index; // index de donde se encuentra la regla de control
  byte state; // actual estado del rele off 0, on 1
}IOcontrolDef;

typedef struct manual_Def_struct {
  byte start_date[5]; // start_date[0]: año, start_date[1]: mes, start_date[2]: dia, ...  Hora y Minute
  byte end_date[5]; // cuando termina la regla
  boolean state; // el estado que fuerza esta regla manual true: encendido, false: apagado
}manualDef;

typedef struct step_def_struct {
  byte step_end;
  byte ref; // referencia, 0: off, 1: on, >1 referencia
}step_def;

typedef struct rules_Def_struct {
  byte steps_num; // numero de steps
  step_def steps[MAX_STEPS_PER_RULE];
  byte devid; // devid de medicion
  byte signal_index;
  boolean polaridad_histeresis;
  byte tol_histeresis; // tolerancia histeresis
  boolean current_actuator; // estado actual empujado por el control (buffer de la histeresis)
}rulesDef;


/* Funciones utils */

/* MACROS */

void keepDatetimeUpdated(); // ask once a day the time to server to keep watches synced
void checkNewRS232Request(); // atiende las peticios por I2C


void sendAck(); // envia un ack al master del canal I2C con el mismo metodo, id y un parametro nulo


void processCtrlPacket(boolean type); // extrae la regla de control y la graba localmente
void processSyncRelayState(); // responde una peticion del servidor de los estados de rele al gateway de gabinete
void keepRelayStateUpdatedonServer(); // envia los datos de los reles periodicamente al servidor 
void sendRelaysState(byte method, byte Id); // envia paquete al servidor con estado de reles

void changeIOstate(byte IO_index, boolean change_to);
void checkControlRules(boolean forced); // evalua las reglas de control
void storeMeasurements(); // guarda las mediciones que llegan enviadas por el gateway

void setDateTime(); // ajusta la hora y fecha
void getDateTime(byte* data); // return *data with current date

void readPacket(int timeToWait); // lee un packete y lo parsea
void sendPacket(); // pack and send
void sendByte(byte b, boolean escape); // send byte

void backupData(boolean isreset); // save control data in EPROM once a day
void restoreData(); // restore control data from gateway, after a while if data has not arrived restore it from in EPROM

/* REad and write anything from eprom */
template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
	  EEPROM.write(ee++, *p++);
    return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
	  *p++ = EEPROM.read(ee++);
    return i;
} 

void printByteArrayDEC(byte* cadena,int len); // print params

void printBuffer(boolean isoutbuffer);

void printIOCtlrDef(byte index);
void printManualDef(byte rule_index);
void printReglaDef(byte rule_index);
void printDate(byte * cadena);
long normDate(byte * cadena);
void cleanEEPROM();
void printList(byte * cadena,byte start_index,byte len);
#endif




