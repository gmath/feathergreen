#include "PietteTech_DHT.h"

/* pins D: digital, A: analog, P: power */
const uint8_t D_dht22pin = 14;
const uint8_t P_sen13322pin = 13;
const uint8_t A_sen13322pin = A0;

/* DHT globals */
#define DHTTYPE DHT22
struct globals_DHT {
  int acquire_result;
  int acquire_status;
  bool b_dht_started;
} g_DHT;

enum {
  DHT_ON_AND_ACQUIRING = 0,
  DHT_OFF,
  DHT_NOT_ACQUIRING
};

/* DHT initializations */
void dht_wrapper();
PietteTech_DHT DHT(D_dht22pin, DHTTYPE, dht_wrapper);
void dht_wrapper() {
  DHT.isrCallback();
}

struct dht22_input {

  double c_temperature;
  double c_humidity;
  double c_dewpt;

};

struct sen13322_input {
  double voltage;
};



int check_dht_status() {

  uint8_t DHT_STATUS = DHT_OFF;
  if ( g_DHT.b_dht_started == true ) {
    
    g_DHT.acquire_status = DHT.acquiring();

    DHT_STATUS = DHT_NOT_ACQUIRING;
    if ( g_DHT.acquire_status == false ) {
      DHT_STATUS = DHT_ON_AND_ACQUIRING; 
    } else {
      g_DHT.b_dht_started = false;
    }
    
  }

  return DHT_STATUS;
}

void get_dht22_input(struct dht22_input * ip_dht22) {
  g_DHT.acquire_result = DHT.getStatus();
  if ( g_DHT.acquire_result == 0 ) {
    ip_dht22->c_temperature = DHT.getCelsius();
    ip_dht22->c_humidity = DHT.getHumidity();
    ip_dht22->c_dewpt = DHT.getDewPoint();
  }
}

void get_sen13322_input(struct sen13322_input *ip_sen13322) {

  /* flow voltage through pins and wait */
  digitalWrite(P_sen13322pin, HIGH);
  delay(10);

  /* read the voltage */
  ip_sen13322->voltage = analogRead(A_sen13322pin) / 1000.0;

  /* turn off the voltage */
  digitalWrite(P_sen13322pin, LOW);
  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  /* dht22 blocking method */
  g_DHT.acquire_status = 0;
  g_DHT.acquire_result = DHT.acquireAndWait(1000);

  /* initialize low voltage on sen13322 soil sensor */
  pinMode(P_sen13322pin, OUTPUT);
  digitalWrite(P_sen13322pin, LOW);

}

void loop() {
  // put your main code here, to run repeatedly:

  /* ------ start handling DHT22 input ----- */
  struct dht22_input i_dht22;
  memset(&i_dht22, 0, sizeof(struct dht22_input));

  int DHT_STATUS = check_dht_status();
  if ( DHT_STATUS == DHT_ON_AND_ACQUIRING )
    get_dht22_input(&i_dht22);

  if ( g_DHT.acquire_status == 1 )
    DHT.reset();

  if ( g_DHT.b_dht_started == false ) {    
    g_DHT.acquire_status = DHT.acquiring();
    g_DHT.b_dht_started = true;
  }
  /* ----- end handling DHT22 input ----- */

  /* ----- start handling sen13322 input ----- */
  struct sen13322_input i_sen13322;
  memset(&i_sen13322, 0, sizeof(struct sen13322_input));

  get_sen13322_input(&i_sen13322);

  /* ----- end handling sen13322 input ----- */


  to_serial(&i_dht22, &i_sen13322);

  delay(2000);
}

void dht22_2_serial(struct dht22_input * ip_dht22) {

  Serial.print(ip_dht22->c_temperature);
  Serial.print(" ");

  Serial.print(ip_dht22->c_humidity);
  Serial.print(" ");

  Serial.print(ip_dht22->c_dewpt);
  Serial.print(" ");
  
  Serial.println();
 
}

void sen13322_2_serial(struct sen13322_input * ip_sen13322) {

  Serial.print(ip_sen13322->voltage);
  Serial.print(" ");
  
  Serial.println();
  
}

void to_serial(struct dht22_input * ip_dht22, struct sen13322_input * ip_sen13322) {

  Serial.print(ip_dht22->c_temperature);
  Serial.print(" ");

  Serial.print(ip_dht22->c_humidity);
  Serial.print(" ");

  Serial.print(ip_dht22->c_dewpt);
  Serial.print(" ");

  Serial.print(ip_sen13322->voltage);
  Serial.print(" ");
  
  Serial.println();
  
}
