#include "Adafruit_MAX31855.h"
#include <LiquidCrystal.h>


// Thermocouple's settings
#define DO 11
#define CS 12
#define CLK 13

Adafruit_MAX31855 thermocouple(CLK, CS, DO);

// Message variables
#define MAX_MSG_SIZE 24

char msg[MAX_MSG_SIZE];
unsigned long start_time = 0;
int n_bytes = 0, i = 0;

// LED to assist troubleshooting; if blinking, then the program is working
#define LED 3
bool pin = false;


void setup(){
  pinMode(LED, OUTPUT);

  Serial.begin(9600);
  
  while(!Serial) delay(1);

  delay(500);
  
  // Initialize thermocouple
  if (!thermocouple.begin()) Serial.println("ERROR");

  // Wait for connection
  while((n_bytes = Serial.available()) < 16) delay(1000);

  // Read message received
  for(i = 0; i < 16; i++) msg[i] = Serial.read();
  msg[i] = '\0';
  // Delete extra characters received
  char aux;
  for(i = 0; i < n_bytes - 16; i++) aux = Serial.read();

  // Check message is correct
  if(strcmp(msg,"READY TO RECEIVE")) Serial.println("ERROR");

  start_time = millis();
}

void loop(){
  // Setup standard message
  sprintf(msg,"T:       t:            \0");

  // Flashing LED to make sure system is working
  if(pin == false){
    digitalWrite(LED, HIGH);
    pin = true;
  }else{
    digitalWrite(LED, LOW);
    pin = false;
  }

  // Read temperature and record respective timestamp
  float c = thermocouple.readCelsius();
  unsigned long current_time = millis() - start_time;

  // Check if value is NAN or not on temperature value
  if(isnan(c)){
    // Timestamp register
    char timestamp[MAX_MSG_SIZE];
    ultoa(current_time, timestamp, 10);
    for(i = 0; i < strlen(timestamp); i++) msg[i + 12] = timestamp[i];

    // NAN temperature value register
    msg[3] = 'N';
    msg[4] = 'A';
    msg[5] = 'N';
    msg[6] = ' ';
    msg[7] = ' ';

  }else{
    // Timestamp register
    char timestamp[MAX_MSG_SIZE];
    ultoa(current_time, timestamp, 10);
    for(i = 0; i < strlen(timestamp); i++) msg[i + 12] = timestamp[i];
    
    // Normal temperature value register
    char temperature[MAX_MSG_SIZE];
    dtostrf(c, 5, 2, temperature);
    for(i = 0; i < strlen(temperature); i++) msg[i + 3] = temperature[i];

  }
  // Send message with data
  Serial.println(msg);

  // Wait for answer on the other end
  while((n_bytes = Serial.available()) < 16) delay(1000);

  // Read message received
  for(i = 0; i < 16; i++) msg[i] = Serial.read();
  msg[i] = '\0';
  // Delete extra characters received
  char aux;
  for(i = 0; i < n_bytes - 16; i++) aux = Serial.read();

  // Check if message is correct
  if(strcmp(msg,"READY TO RECEIVE")) Serial.println("ERROR");

  // Wait a bit for next data point extraction
  delay(100);
}
