/*
 * NOVApcr
 * Author: Francisco Branco
 * Date: 04/07/2021
 * Description: Thermocycler program to control the machine and receive incoming LAMP restricted pcr programs. Also cooperates with a control_probe.
 *              See documentation on github: https://github.com/NeurogeneLocomotion/NOVApcr
 *              
 */

 
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>
#include <MemoryFree.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "Adafruit_MAX31855.h"




// Pins
#define L_HP 51 // Pin for lower heating pad

// Temperature measurement
#define T_DO 12
#define T_CLK 13

#define T1_CS A1   // Sensor 1
#define T2_CS A2   // Sensor 2

#define HP_C 2  // Lower heating pad Control
#define U_HP 52 // Upper heating pad control
#define BUZZ 11 // Buzzer for end of program

//LCD pin to Arduino
#define RS 8
#define EN 9
#define D4 4
#define D5 5
#define D6 6
#define D7 7
#define BL 10
#define BUTTON A0




LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

byte fillChar[] = {
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};



/*  Bluetooth Block
     -
*/
#define BUFFER_SIZE 22  // Message Size
#define NUM_PARAM 4   // Number of parameters on each line
char msg[BUFFER_SIZE];  // Message buffer
char auxChar = '\0';  // Auxiliary character
byte byte_count = 0;
int first_bytes = 0, remaining_bytes = 0;
int N_lines = 0;



/*  Temperature Block
     - Define pins
     - Setup Library or calculations for temperature
*/
Adafruit_MAX31855 sensor1(T_CLK, T1_CS, T_DO);
Adafruit_MAX31855 sensor2(T_CLK, T2_CS, T_DO);




/*  PID Block
     - Define value range
     - Initialize variables
     - PID gains
     - Setup PID
*/
#define OUTPUT_MIN 0  // PID Constants
#define OUTPUT_MAX 12
double input, setpoint, output; // PID parameters
double kp = 100, ki = 80, kd = 80;  // PID constants kp = 1, ki = 0.5, kd = 0.9
PID tempPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  // PID setup



// State and Major variables
int state = 0, duration = 0, line = 0, PID_state = 0, total_time = 0, curr_time = 0, flag = 1, curr_goto = -1, connection_type = 0, prev_stage[2] = {0}, index = 0, buzzer = 0, c_nan[2] = {0}, temp_diff = 0;
double current_temperature = 0, previous_temperature = 0, temp2 = 0, value = 0;
unsigned long starting_time = 0, meantime = millis(), difference = 0, pause_time = 0;

// Command allocation matrix
int cmd_matrix[50][NUM_PARAM] = {0};






void setup() {
  tone(BUZZ, 784);
  
  Serial.begin(9600);
  Serial.setTimeout(1);

  Serial1.begin(9600);
  Serial1.setTimeout(1);

  delay(1000);

  pinMode(HP_C, OUTPUT); // Pin for lower heating pad control

  pinMode(L_HP, OUTPUT);  // Pins for lower heating pad connection

  pinMode(U_HP, OUTPUT);  // Pin for Heat Pad control
  pinMode(BUZZ, OUTPUT);  // Pin for Buzzer

  sensor1.begin();  // Starting temperature sensors
  sensor2.begin();

  lcd.begin(16, 2);
  lcd.createChar(0, fillChar);

  setpoint = 0;
  PID_state = 0;
  output = 0;

  tempPID.SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  tempPID.SetMode(AUTOMATIC);

  int address = 0;
  EEPROM.get(address, N_lines);
  address += sizeof(int);
  for(int i = 0; i < N_lines * 4; i++){
    address += sizeof(int);
    if (address >= EEPROM.length()) {
      lcd.clear();
      lcd.home();
      lcd.write("NAME LOAD ERROR");
      lcd.setCursor(0, 1);
      lcd.write("RESET AND RELOAD");
      while (1) delay(1000);
    }
  }
  for(int i = 0; i < 17; i++){
    EEPROM.get(address, msg[i]);
    address += sizeof(char);
    if (address >= EEPROM.length()) {
      lcd.clear();
      lcd.home();
      lcd.write("NAME LOAD ERROR");
      lcd.setCursor(0, 1);
      lcd.write("RESET AND RELOAD");
      while (1) delay(1000);
    }
  }

  noTone(BUZZ);
}




void loop() {
  // Take current temperatures
  double new_c1 = sensor1.readCelsius(), new_c2 = sensor2.readCelsius();

  

  if (!isnan(new_c1)) current_temperature = new_c1;
  if (!isnan(new_c2)) temp2 = new_c2;

  if(state > 0) verify_temp(new_c1, new_c2);

  if (state == 0) {
    // Reception state
    receive_message();
    if (connection_type) {
      lcd.clear();
      lcd.home();
      lcd.print("NEW PROGRAM INC.");
      lcd.setCursor(0, 1);
      lcd.print("PLEASE WAIT...");
    } else {
      lcd.home();
      lcd.print("PRESS START");
      lcd.setCursor(0, 1);
      lcd.print(msg);
      //Serial.println(analogRead(BUTTON));
      if (analogRead(BUTTON) > 600 && analogRead(BUTTON) < 800) {
        tone(BUZZ, 523, 1500);
        delay(1600);
        noTone(BUZZ);
        
        load_program();

        setup_gotos();
        calculate_total_time(0);
        setup_gotos();
        setup_temp();
        curr_goto = calculate_curr_goto();

        line = 1;
        flag = 1;
        buzzer = 1;
        difference = 0;
        PID_state = 1;

        Serial.println("START PROGRAM");
        Serial1.println("START PROGRAM");

        // Heating pad pre-heating
        lcd.clear();
        lcd.home();
        lcd.write("HP: ");
        char hp_temp_str[10];
        lcd.setCursor(9, 0);
        lcd.write(" C");
        lcd.setCursor(0, 1);
        lcd.write("PLEASE HOLD");
        digitalWrite(U_HP, HIGH); // Turn on Upper Heat Pad
        digitalWrite(L_HP, HIGH); // Turn on Lower Heat Pad
        analogWrite(HP_C, 0);
        
        while(1){
          new_c1 = sensor1.readCelsius();
          new_c2 = sensor2.readCelsius();
          
          if(!isnan(new_c1)) current_temperature = new_c1;
          if(!isnan(new_c2)) temp2 = new_c2;
          verify_temp(new_c1, new_c2);
          lcd.setCursor(4, 0);
          lcd.write("      C");
          lcd.setCursor(4, 0);
          dtostrf(temp2, 5, 1, hp_temp_str);
          lcd.write(hp_temp_str);
          if(temp2 >= 65.0) break;

          delay(1);
        }

        state = 1;
      }
    }

  } else if (state == 1) {
    // Command selection function
    select_func();
    flag = 0;
    previous_temperature = current_temperature;

  } else if (state == 2) {
    // Transient state
    if (current_temperature > setpoint - 1 && current_temperature < setpoint + 1) {
      state = 3;
      starting_time = millis();
    }

  } else if (state == 3 && millis() - starting_time >  ((unsigned long) duration) * ((unsigned long) 1000) - difference) {
    // Normal Timer state
    curr_time += cmd_matrix[line++ - 1][2];
    flag = 0;
    state = 1;

  } else if (state == 4) {
    // Pause state
    if (!flag) {
      lcd.home();
      lcd.print("PAUSING");
      lcd.setCursor(0, 1);
      lcd.print("TIME: ");

      flag = 1;
    }
    lcd.setCursor(6, 1);
    lcd.print((int) ((millis() - pause_time) / 1000));

  } else if (state == 5) {    
    // End of program state
    digitalWrite(U_HP, LOW);
    if(buzzer){
      buzzer = 0;
      for(int j = 0; j < 3; j++){
        tone(BUZZ, 440, 1000);
        delay(1100);
        noTone(BUZZ);
        delay(500);
      }
    }
  }

  // Check if select button is pushed while program is running
  if (analogRead(BUTTON) > 600 && analogRead(BUTTON) < 800 && state != 0) {
    tone(BUZZ, 784, 500);
    delay(600);
    noTone(BUZZ);
    if (state > 1 && state < 4) {
      if (state == 3) difference += millis() - starting_time;
      pause_time = millis();

      prev_stage[0] = state;
      prev_stage[1] = setpoint;

      lcd.clear();
      flag = 0;
      state = 4;
      setpoint = current_temperature;
    } else if (state == 4) {
      state = prev_stage[0];
      setpoint = prev_stage[1];

      // Flag to zero so display clears
      flag = 0;
      if (state == 3) starting_time = millis();

      PID_state = 1;
    } else if (state == 5) {
      pause_time = millis();

      prev_stage[0] = state;
      prev_stage[1] = setpoint;

      if (buzzer) buzzer = 0;

      state = 4;
      PID_state = 0;
    }
  }


  // PID_state on = regulating temperature, otherwise do nothing
  if (PID_state) {
    control_func();
  } else {
    digitalWrite(L_HP, LOW);
    analogWrite(HP_C, 255);
  }

  // Display loop properties
  if ((state > 0 && state < 4) || state == 5) {
    loop_display();
  }

  delay(100);
}








// Function to verify if temperature is nan for a while
void verify_temp(int temp1, int temp2) {
  // Verify nan
  if (isnan(temp1)) c_nan[0]++;
  else c_nan[0] = 0;
  if (isnan(temp2)) c_nan[1]++;
  else c_nan[1] = 0;
  if (c_nan[0] > 100 || c_nan[1] > 100) {
    lcd.clear();
    lcd.home();
    lcd.print("MALFUNCTION");
    lcd.setCursor(0, 1);
    lcd.print("TOO MANY NAN T");
    if (c_nan[0] > 100) lcd.print("1");
    else lcd.print("2");

    digitalWrite(L_HP, LOW);
    digitalWrite(U_HP, LOW);
    analogWrite(HP_C, 255);

    while (1) delay(1);
  }

  if (temp2 < 60.0 && state != 5 && state != 0) {
    if (temp_diff++ > 100) {
      char aux_str[10];
      lcd.clear();
      lcd.home();
      lcd.print("MALFUNCTION");
      lcd.setCursor(0, 1);
      lcd.print("TC:");
      if (current_temperature > 99.9) sprintf(aux_str, ">+100");
      else if (current_temperature < -99.9) sprintf(aux_str, "<-100");
      else dtostrf(current_temperature, 5, 1, aux_str);
      lcd.print(aux_str);
      lcd.print("HP:");
      if (temp2 > 99.9) sprintf(aux_str, ">+100");
      else if (temp2 < -99.9) sprintf(aux_str, "<-100");
      else dtostrf(temp2, 5, 1, aux_str);
      lcd.print(aux_str);
      
      digitalWrite(L_HP, LOW);
      digitalWrite(U_HP, LOW);
      analogWrite(HP_C, 255);

      while (1) delay(1);
    }
  }else {
    temp_diff = 0;
  }
}




// Function that reads the command matrix and selects the command
void select_func(){
  double temp_val, inc_val;
  // Choose between STEP, GOTO and END
  switch (cmd_matrix[line - 1][0]) {
    // For STEP: set temperature and duration
    case 0:
      temp_val = (double) cmd_matrix[line - 1][1];
      inc_val = (double) cmd_matrix[line - 1][3];
      setpoint = (temp_val + inc_val) / 10.0;
      starting_time = 0;
      duration = cmd_matrix[line - 1][2];
      state = 2;
      break;

    // For GOTO
    case 1:
      if (--cmd_matrix[line - 1][3]) {
        line = cmd_matrix[line - 1][1];
        select_func();
      } else {
        line++;
        curr_goto = calculate_curr_goto();
        select_func();
      }
      break;

    // For END
    case 2:
      temp_val = (double) cmd_matrix[line - 1][1];
      setpoint = temp_val / 10.0;
      meantime = millis();
      state = 5;
      lcd.clear();
      break;
    default:
      break;
  }
}



void control_func(){
  input = current_temperature;

  tempPID.Compute();

  value = abs(255 * output / 12);

  analogWrite(HP_C, 255 - value);
  if(value == 0 ) digitalWrite(L_HP, LOW);

  if (state == 5) {
    digitalWrite(U_HP, LOW);
    digitalWrite(L_HP, LOW);
  } else {
    // Upper heating pad control
    if (state > 0 && state < 5){
      if (temp2 > 75){
        digitalWrite(U_HP, LOW);
      }else if (temp2 < 70){
        digitalWrite(U_HP, HIGH);
      }
    }
    digitalWrite(L_HP, HIGH);
  }
}



// Function to fill out each line of the command matrix (this part of the implementation highly depends on the number of parameters this project is using - 5 parameters)
void fill_out(int param_size[NUM_PARAM], int param_offset[NUM_PARAM]) {
  int i = 0, j = 0, aux = 0;
  // The size of this array has to be the maximum number in para_size vector
  char aux_Array[5] = {" "};
  aux_Array[4] = '\0';

  for (i = 0; i < NUM_PARAM + 1; i++) {
    for (j = 0; j < param_size[i]; j++) aux_Array[j] = msg[j + param_offset[i]];
    aux_Array[j] = '\0';

    if (i == 0) {
      aux = atoi(aux_Array);

      if (aux != index + 1 || index >= N_lines) {
        //Serial.println(index);
        //Serial.print
        lcd.clear();
        lcd.home();
        lcd.write("MESSAGE ERROR");
        lcd.setCursor(0, 1);
        lcd.write("RESET AND RELOAD");
        while (1) delay(1000);
      }
      index++;
    } else {
      cmd_matrix[index - 1][i - 1] = atoi(aux_Array);
    }
  }
}



// Function which receives the incoming message
void receive_message() {
  int i = 0;
  char auxChar;
  // These numbers below are dependent on the number and size of each parameter established on the computer side
  int param_size[] = {4, 1, 4, 4, 4}, param_offset[] = {0, 5, 7, 12, 17};
  //byte_count = bt_connection.available();
  byte_count = Serial1.available();
  if (byte_count >= BUFFER_SIZE - 1) {
    if (connection_type == 0) connection_type = 1;
  } else {
    byte_count = Serial.available();
    if (byte_count >= BUFFER_SIZE - 1) if (connection_type == 0) connection_type = 2;
  }

  if (byte_count >= BUFFER_SIZE - 1 && connection_type) {

    // Incoming Message
    first_bytes = byte_count;
    remaining_bytes = byte_count - (BUFFER_SIZE - 1);

    for (i = 0; i < first_bytes; i++) {
      //if(connection_type == 1) auxChar = (char) bt_connection.read();
      if (connection_type == 1) auxChar = (char) Serial1.read();
      else if (connection_type == 2) auxChar = (char) Serial.read();
      msg[i] = auxChar;
    }

    msg[i] = '\0';
    auxChar = msg[11];
    msg[11] = '\0';

    if (strcmp(msg, "NEW PROGRAM") == 0) {
      char chr_lines[5];
      index = 0;
      chr_lines[4] = '\0';
      for (i = 0; i < 4; i++) chr_lines[i] = msg[12 + i];
      N_lines = atoi(chr_lines);
      flag = 1;
      lcd.clear();
    }
    msg[11] = auxChar;
    msg[4] = '\0';

    if (strcmp(msg, "NAME") == 0) {
      msg[4] = ' ';

      if (N_lines != index) {
        lcd.clear();
        lcd.home();
        lcd.write("MESSAGE ERROR");
        lcd.setCursor(0, 1);
        lcd.write("RESET AND RELOAD");
        while (1) delay(1000);
      }

      for (i = 0; i < 16; i++) msg[i] = msg[i + 5];
      msg[16] = '\0';

      save_program();

      connection_type = 0;
      flag = 1;
      lcd.clear();

      tone(BUZZ, 587, 500);
      delay(600);
      noTone(BUZZ);
      tone(BUZZ, 587, 500);
      delay(600);
      noTone(BUZZ);
    }

    if (!flag) {
      fill_out(param_size, param_offset);
    }

    for (i = 0; i < remaining_bytes; i++) {
      //if(connection_type == 1) auxChar = bt_connection.read();
      if (connection_type == 1) auxChar = Serial1.read();
      else if (connection_type == 2) auxChar = Serial.read();
    }

    // Send answer back to computer
    //if(connection_type == 1) bt_connection.println("READY TO RECEIVE");
    if (connection_type == 1) Serial1.println("READY TO RECEIVE");
    else if (connection_type == 2) Serial.println("READY TO RECEIVE");
    flag = 0;
  }
}



void calculate_total_time(int curr_line) {
  switch (cmd_matrix[curr_line][0]) {
    case 0:

      total_time += cmd_matrix[curr_line][2];
      calculate_total_time(curr_line + 1);
      break;
    case 1:
      if (--cmd_matrix[curr_line][3]) calculate_total_time(cmd_matrix[curr_line][1] - 1);
      else calculate_total_time(curr_line + 1);
      break;
    default:
      break;
  }
}



void setup_gotos() {
  // Fill out goto extra slot and setup temperature for cycles
  for (int i = 0; i < N_lines; i++) {
    if (cmd_matrix[i][0] == 1) cmd_matrix[i][3] = cmd_matrix[i][2];
  }
}



void setup_temp() {
  for (int i = 0; i < N_lines; i++) {
    if (cmd_matrix[i][0] == 0) cmd_matrix[i][1] -= cmd_matrix[i][3];
  }
}



int calculate_curr_goto() {
  for (int i = 0; i < N_lines; i++) if (cmd_matrix[i][0] == 1 && i > curr_goto) return i;
  return -1;
}



void loop_display() {
  char aux_str[20];
  if ((analogRead(BUTTON) > 400 && analogRead(BUTTON) < 600) && state != 5) {
    tone(BUZZ, 784, 500);
    delay(600);
    noTone(BUZZ);
    if (flag < 2) {
      flag = 2;
      meantime = millis();
    } else {
      flag = 0;
    }
  }

  if (flag < 2) {
    lcd.setCursor(2, 0);
    lcd.print("    ");
    lcd.setCursor(2, 0);
    if (current_temperature > 99.9) sprintf(aux_str, ">100\0");
    else if (current_temperature < -99.9) sprintf(aux_str, "<-100\0");
    else dtostrf(current_temperature, 5, 1, aux_str);
    lcd.print(aux_str);
    if (state != 5) {
      lcd.setCursor(2, 1);
      lcd.print("    ");
      lcd.setCursor(2, 1);
    }
    if (state == 2) lcd.print(0);
    else if (state != 5) lcd.print((int) ((millis() - starting_time) / 1000));
    if (flag == 0) {
      lcd.clear();
      lcd.home();
      lcd.print("T:");

      lcd.setCursor(7, 0);
      lcd.print("/");
      dtostrf(setpoint, 5, 1, aux_str);
      lcd.print(aux_str);
      lcd.print("C");
      lcd.setCursor(0, 1);
      if (state != 5) {
        lcd.print("t:");

        lcd.setCursor(7, 1);
        lcd.print("/");
        lcd.print(duration);
        lcd.print("s");
      } else {
        lcd.setCursor(0, 1);
        lcd.print("REACHED END");
      }

      flag = 1;
    }
  } else {
    lcd.home();
    if (millis() - meantime < 3000) {
      lcd.print("                ");
      lcd.home();
      lcd.print(msg);
      lcd.setCursor(5, 1);
      lcd.print("    ");
      lcd.setCursor(5, 1);
      if (curr_goto >= 0) lcd.print(cmd_matrix[curr_goto][2] - cmd_matrix[curr_goto][3] + 1);
      else lcd.print(0);
      lcd.setCursor(12, 1);
      lcd.print("    ");
      lcd.setCursor(12, 1);
      lcd.print(line);
  
      if (flag == 2) {
        lcd.setCursor(0, 1);
        lcd.print("Cycl:");
  
        lcd.setCursor(9, 1);
        lcd.print(" L:");
  
        flag = 3;
      }
    } else if (millis() - meantime < 6000) {
      lcd.print("|");
      for (int i = 0; i < 14; i++) {
        lcd.setCursor(i + 1, 0);
        if (map(curr_time, 0, total_time, 0, 14) >= i) lcd.write((byte) 0);
        else lcd.print(" ");
      }
      lcd.setCursor(6, 0);
      lcd.print(map(curr_time, 0, total_time, 0, 100));
      lcd.setCursor(9, 0);
      lcd.print("%");
      lcd.setCursor(15, 0);
      lcd.print("|");

      if(flag == 2){
        lcd.setCursor(0, 1);
        lcd.print("TH:        C");
      }
      

      if (temp2 > 99.9) sprintf(aux_str, ">100\0");
      else if (temp2 < -99.9) sprintf(aux_str, "<-100\0");
      else dtostrf(temp2, 5, 1, aux_str);

      lcd.setCursor(3, 1);
      lcd.print(aux_str);
    } else {
      meantime = millis();
    }
  }
}



void save_program() {
  int address = 0, reach_end = 0;
  
  EEPROM.put(address, N_lines);
  address += sizeof(int);

  if (address >= EEPROM.length()) {
    lcd.clear();
    lcd.home();
    lcd.write("SAVE ERROR");
    lcd.setCursor(0, 1);
    lcd.write("RESET AND RELOAD");
    while (1) delay(1000);
    return;
  }

  for (int i = 0; i < N_lines; i++) {
    for (int j = 0; j < 4; j++) {
      EEPROM.put(address, cmd_matrix[i][j]);
      address += sizeof(int);

      if (address >= EEPROM.length()) {
        lcd.clear();
        lcd.home();
        lcd.write("SAVE ERROR");
        lcd.setCursor(0, 1);
        lcd.write("RESET AND RELOAD");
        while (1) delay(1000);
        return;
      }
    }
  }
  for (int i = 0; i < 17; i++) {
    EEPROM.put(address, msg[i]);
    address += sizeof(char);

    if (address >= EEPROM.length()) {
      lcd.clear();
      lcd.home();
      lcd.write("SAVE ERROR");
      lcd.setCursor(0, 1);
      lcd.write("RESET AND RELOAD");
      while (1) delay(1000);
      return;
    }
  }
}



void load_program() {
  int address = 0, reached = 0;
  line = 0;

  EEPROM.get(address, N_lines);
  address += sizeof(int);
  
  while (1) {
    for (int i = 0; i < 4; i++) {
      EEPROM.get(address, cmd_matrix[line][i]);
      address += sizeof(int);

      if (address >= EEPROM.length()) {
        lcd.clear();
        lcd.home();
        lcd.write("LOAD ERROR");
        lcd.setCursor(0, 1);
        lcd.write("RESET AND RELOAD");
        while (1) delay(1000);
        //return;
      }
    }
    if (cmd_matrix[line++][0] == 2) break;
  }



  for (int i = 0; i < 17; i++) {
    EEPROM.get(address, msg[i]);
    address += sizeof(char);
    if (address >= EEPROM.length()) {
      lcd.clear();
      lcd.home();
      lcd.write("LOAD ERROR");
      lcd.setCursor(0, 1);
      lcd.write("RESET AND RELOAD");
      while (1) delay(1000);
      //return;
    }
  }
}



void debug_print_matrix() {

  for (int i = 0; i < N_lines; i++) {
    Serial.print("line: ");
    Serial.print(i);
    for (int j = 0; j < 4; j++) {
      Serial.print(", Value: ");
      Serial.print(cmd_matrix[i][j]);
    }
    Serial.println();
  }
}
