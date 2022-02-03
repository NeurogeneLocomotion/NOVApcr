/*
 * NOVApcr
 * Author: Francisco Branco
 * Date: 04/07/2021
 * Description: Thermocycler program to control the machine and receive incoming pcr programs. Also cooperates with a control_probe.
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
// Peltier pins have a direct and inverse current flow thanks to relays
#define P1DIR1 51  // Peltier 1 signals direction
#define P1DIR2 49
#define P1INV1 50
#define P1INV2 48

#define P2DIR1 47  // Peltier 2 signals direction
#define P2DIR2 44
#define P2INV1 45
#define P2INV2 46

#define FAN 53  // Fan signal

// Temperature measurement
#define T_DO 12
#define T_CLK 13

#define T1_CS A1   // Sensor 1
#define T2_CS A2   // Sensor 2
#define T3_CS A3   // Sensor 3
#define T4_CS A4   // Sensor 4

#define PELT1 2  // Peltier Output Control
#define PELT2 3

#define HP 52 // Heating pad control
#define BUZZ 11 // Buzzer

//LCD pins to Arduino
#define RS 8
#define EN 9
#define D4 4
#define D5 5
#define D6 6
#define D7 7
#define BL 10
#define BUTTON A0



// LCD initialization
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

// Character for loading bar
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
     - Setup all the variables that control and execute the communication of the pcr programs with the machine
*/
#define BUFFER_SIZE 22  // Message Size
#define NUM_PARAM 4   // Number of parameters on each line
char msg[BUFFER_SIZE];  // Message buffer
char auxChar = '\0';  // Auxiliary character
byte byte_count = 0;  // To check if there are any incoming bytes
int first_bytes = 0, remaining_bytes = 0; // These are used to read the message
int N_lines = 0;  // Number of total pcr program lines



/*  Temperature Block
     - Initialize sensors for temperature measurement
     - Each sensor shares the CLK (clock) and DO (data output) pins; the CS (chip select) changes for each one
*/
Adafruit_MAX31855 sensor1(T_CLK, T1_CS, T_DO);  // Sensor 1 for peltier 1
Adafruit_MAX31855 sensor2(T_CLK, T2_CS, T_DO);  // Sensor 2 for peltier 2
Adafruit_MAX31855 sensor3(T_CLK, T3_CS, T_DO);  // Sesnor 3 for heating pad monitoring
Adafruit_MAX31855 sensor4(T_CLK, T4_CS, T_DO);  // Sensor 4 for probe monitoring special implementation (see documentation for further insight)




/*  PID Block
    - Define value range
    - Initialize parameters
    - PID gains
    - Setup PID
*/
#define OUTPUT_MIN -12  // PID Constants
#define OUTPUT_MAX 12
double input, setpoint, output; // PID parameters
double kp = 100, ki = 80, kd = 80;
PID tempPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);  // PID setup



/*
 * State and major variables
 *  - State 0: ready to receive messages/main menu
 *  - State 1: reading next command from pcr program
 *  - State 2: reaching the specified temperature
 *  - State 3: waiting for the duration of specified step
 *  - State 4: pausing pcr program
 *  - State 5: reached end of program
 */
int state = 0;

int duration = 0; // Carries the duration of the current step
int line = 0; // The current line of the pcr program
int PID_state = 0;  // Tells if the control is currently engaged or not
int total_time = 0; // Total time in seconds of the pcr program
int curr_time = 0;  // Elapsed time of the pcr program
int flag = 1; // Flag used for several messaging properties and lcd interface/state
int curr_goto = -1; // Indicates on which goto loop the pcr program is currently
int connection_type = 0;  // Records if connection is bluetooth or usb
int prev_stage[2] = {0};  // Saves the current state and temperature of pcr program for pausing purposes
int index = 0;  // Current index (similar to line) of the pcr program
int buzzer = 0; // Indicates if buzzer is on/off
int c_nan[4] = {0}; // Captures how many consecutive NaN values the temperature probe reads for troubleshooting
int temp_diff = 0;  // If temperature difference between the two peltiers is very different

// c1 and c2 are auxiliary variables to read temperature values; temp3 is the temperature of the heating pad; value is the variable used for the PID output
double current_temperature = 0, previous_temperature = 0, c1 = 0, c2 = 0, temp3 = 0, value = 0;

// Variables regarding time related parameters
unsigned long starting_time = 0, meantime = millis(), difference = 0, pause_time = 0;

// pcr program command allocation matrix
int cmd_matrix[50][NUM_PARAM] = {0};






void setup() {
  tone(BUZZ, 784);

  Serial.begin(9600);
  Serial.setTimeout(1);

  Serial1.begin(9600);
  Serial1.setTimeout(1);

  delay(1000);

  pinMode(PELT1, OUTPUT); // Pin for peltier control
  pinMode(PELT2, OUTPUT);

  pinMode(P1DIR1, OUTPUT);  // Pins for direct peltier
  pinMode(P1DIR2, OUTPUT);
  pinMode(P2DIR1, OUTPUT);
  pinMode(P2DIR2, OUTPUT);
  pinMode(P1INV1, OUTPUT);  // Pins for inverting peltier
  pinMode(P1INV2, OUTPUT);
  pinMode(P2INV1, OUTPUT);
  pinMode(P2INV2, OUTPUT);

  pinMode(FAN, OUTPUT);  // Pin for fan control
  pinMode(HP, OUTPUT);  // Pin for Heat Pad control
  pinMode(BUZZ, OUTPUT);  // Pin for Buzzer

  sensor1.begin();  // Starting temperature sensors
  sensor2.begin();
  sensor3.begin();
  //sensor4.begin();

  lcd.begin(16, 2);
  lcd.createChar(0, fillChar);

  setpoint = 0;
  PID_state = 0;
  output = 0;

  tempPID.SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  tempPID.SetMode(AUTOMATIC);

  // Reading saved pcr program
  int address = 0;
  EEPROM.get(address, N_lines);
  address += sizeof(int);
  for (int i = 0; i < N_lines * 4; i++) {
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
  for (int i = 0; i < 17; i++) {
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
  double new_c1 = sensor1.readCelsius(), new_c2 = sensor2.readCelsius(), new_c3 = sensor3.readCelsius(); //new_c4 = sensor4.readCelsius();


  // Check temperature read to present consistent numbers for lcd and calculations
  if (isnan(new_c1) && !isnan(new_c2)) {
    current_temperature = (c1 + new_c2) / 2.0;
    c2 = new_c2;
  } else if (!isnan(new_c1) && isnan(new_c2)) {
    current_temperature = (new_c1 + c2) / 2.0;
    c1 = new_c1;
  } else if (!isnan(new_c1) && !isnan(new_c2)) {
    current_temperature = (new_c1 + new_c2) / 2.0;
    c1 = new_c1;
    c2 = new_c2;
  } else {
    current_temperature = (c1 + c2) / 2.0;
  }

  /*
    if(!isnan(new_c1)) c1 = new_c1;
    if(!isnan(new_c2)) c2 = new_c2;
    if(!isnan(new_c1)) current_temperature = new_c4;*/
  
  if (!isnan(new_c3)) temp3 = new_c3;
  
  if (state > 0) verify_temp(new_c1, new_c2, new_c3);

  // Start state checking in the main loop
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
        double hp_temp = sensor3.readCelsius(), hp_temp_;
        while (isnan(hp_temp)) hp_temp = sensor3.readCelsius();
        dtostrf(hp_temp, 5, 1, hp_temp_str);
        lcd.setCursor(4, 0);
        lcd.write(hp_temp_str);
        lcd.setCursor(9, 0);
        lcd.write(" C");
        lcd.setCursor(0, 1);
        lcd.write("PLEASE HOLD");
        digitalWrite(HP, HIGH); // Turn on Heat Pad
        c1 = 0;
        c2 = 0;
        current_temperature = 0;

        while (1) {
          hp_temp_ = sensor3.readCelsius();
          if (!isnan(hp_temp_)) hp_temp = hp_temp_;
          verify_temp(0, 0, hp_temp_);
          lcd.setCursor(4, 0);
          lcd.write("      C");
          lcd.setCursor(4, 0);
          dtostrf(hp_temp, 5, 1, hp_temp_str);
          lcd.write(hp_temp_str);
          if (hp_temp >= 65.0) break;
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
    // Normal Timer state (step)
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
    digitalWrite(HP, LOW);
    if (buzzer) {
      buzzer = 0;
      for (int j = 0; j < 3; j++) {
        tone(BUZZ, 440, 1000);
        delay(1100);
        noTone(BUZZ);
        delay(500);
      }
    }
  }
  

  // Check if select button is pushed while program is running (pausing)
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
    peltier_direction(0);
    analogWrite(PELT1, 255);
    analogWrite(PELT2, 255);
  }

  // Display loop properties
  if ((state > 0 && state < 4) || state == 5) {
    loop_display();
  }

  // Upper heating pad control
  if (state > 0 && state < 5) {
    if (temp3 > 75) {
      digitalWrite(HP, LOW);
    } else if (temp3 < 70) {
      digitalWrite(HP, HIGH);
    }
  }

  delay(100);
}








// Function to verify if temperature is nan for a while
void verify_temp(int temp1, int temp2, int _temp3) {
  // Verify nan
  if (isnan(temp1)) c_nan[0]++;
  else c_nan[0] = 0;
  if (isnan(temp2)) c_nan[1]++;
  else c_nan[1] = 0;
  if (isnan(_temp3)) c_nan[2]++;
  else c_nan[2] = 0;
  /*if (isnan(temp4)) c_nan[3]++;
    else c_nan[3] = 0;*/
  if (c_nan[0] > 100 || c_nan[1] > 100 || c_nan[2] > 100 /*|| c_nan[3] > 100*/) {
    lcd.clear();
    lcd.home();
    lcd.print("MALFUNCTION");
    lcd.setCursor(0, 1);
    lcd.print("TOO MANY NAN T");
    if (c_nan[0] > 100) lcd.print("1");
    else if (c_nan[1] > 100) lcd.print("2");
    else if (c_nan[2] > 100) lcd.print("3");
    /*else lcd.print("4");*/

    peltier_direction(0);
    analogWrite(PELT1, 255);
    analogWrite(PELT2, 255);

    while (1) delay(1);
  }

  if ((c1 < c2 - 10 || c1 > c2 + 10) && state > 0) {
    if (temp_diff++ > 100) {
      char aux_str[10];
      lcd.clear();
      lcd.home();
      lcd.print("MALFUNCTION");
      lcd.setCursor(0, 1);
      lcd.print("T1:");
      if (c1 > 99.9) sprintf(aux_str, ">+100");
      else if (c1 < -99.9) sprintf(aux_str, "<-100");
      else dtostrf(c1, 5, 1, aux_str);
      lcd.print(aux_str);
      lcd.print("T2:");
      if (c2 > 99.9) sprintf(aux_str, ">+100");
      else if (c2 < -99.9) sprintf(aux_str, "<-100");
      else dtostrf(c2, 5, 1, aux_str);
      lcd.print(aux_str);

      peltier_direction(0);
      analogWrite(PELT1, 255);
      analogWrite(PELT2, 255);

      while (1) delay(1);
    }
  } else if (temp3 < 60.0 && state != 5 && state != 0) {
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
      if (temp3 > 99.9) sprintf(aux_str, ">+100");
      else if (temp3 < -99.9) sprintf(aux_str, "<-100");
      else dtostrf(temp3, 5, 1, aux_str);
      lcd.print(aux_str);

      peltier_direction(0);
      analogWrite(PELT1, 255);
      analogWrite(PELT2, 255);

      while (1) delay(1);
    }
  } else {
    temp_diff = 0;
  }
}




// Function that turns the direction of the peltiers
void peltier_direction(int direction) {
  if (direction > 0) {
    digitalWrite(P1INV1, LOW);
    digitalWrite(P2INV1, LOW);
    digitalWrite(P1DIR1, HIGH);
    digitalWrite(P2DIR1, HIGH);
    digitalWrite(P1INV2, LOW);
    digitalWrite(P2INV2, LOW);
    digitalWrite(P1DIR2, HIGH);
    digitalWrite(P2DIR2, HIGH);
    digitalWrite(FAN, LOW);
  } else if (direction < 0) {
    digitalWrite(P1INV1, HIGH);
    digitalWrite(P2INV1, HIGH);
    digitalWrite(P1DIR1, LOW);
    digitalWrite(P2DIR1, LOW);
    digitalWrite(P1INV2, HIGH);
    digitalWrite(P2INV2, HIGH);
    digitalWrite(P1DIR2, LOW);
    digitalWrite(P2DIR2, LOW);
    digitalWrite(FAN, HIGH);
  } else {
    digitalWrite(P1INV1, LOW);
    digitalWrite(P2INV1, LOW);
    digitalWrite(P1DIR1, LOW);
    digitalWrite(P2DIR1, LOW);
    digitalWrite(P1INV2, LOW);
    digitalWrite(P2INV2, LOW);
    digitalWrite(P1DIR2, LOW);
    digitalWrite(P2DIR2, LOW);
    digitalWrite(FAN, LOW);
  }
}




// Function that reads the command matrix and selects the command
void select_func() {
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


// For PID control of peltiers
void control_func() {
  input = current_temperature;

  tempPID.Compute();

  value = abs(255 * output / 12);
  
  peltier_direction(output);
  analogWrite(PELT1, 255 - value);
  analogWrite(PELT2, 255 - value);

  // Additional heating pad relay control
  if (state == 5) {
    digitalWrite(HP, LOW);
  } else {
    digitalWrite(HP, HIGH);
  }
}



// Function to fill out each line of the command matrix (this part of the implementation highly depends on the number of parameters this project is using - 5 parameters)
void fill_out(int param_size[NUM_PARAM], int param_offset[NUM_PARAM]) {
  int i = 0, j = 0, aux = 0;
  // The size of this array has to be the maximum number in param_size vector
  char aux_Array[5] = {" "};
  aux_Array[4] = '\0';

  for (i = 0; i < NUM_PARAM + 1; i++) {
    for (j = 0; j < param_size[i]; j++) aux_Array[j] = msg[j + param_offset[i]];
    aux_Array[j] = '\0';

    if (i == 0) {
      aux = atoi(aux_Array);

      if (aux != index + 1 || index >= N_lines) {
        
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
      
      if (connection_type == 1) auxChar = Serial1.read();
      else if (connection_type == 2) auxChar = Serial.read();
    }

    // Send answer back to computer
    if (connection_type == 1) Serial1.println("READY TO RECEIVE");
    else if (connection_type == 2) Serial.println("READY TO RECEIVE");
    flag = 0;
  }
}




// Functions that calculates the total amount of time the pcr program takes to finish; useful for loading functionality
void calculate_total_time(int curr_line) {
  switch (cmd_matrix[curr_line][0]) {
    // Step case
    case 0:
      total_time += cmd_matrix[curr_line][2];
      calculate_total_time(curr_line + 1);
      break;
    // Goto case
    case 1:
      if (--cmd_matrix[curr_line][3]) calculate_total_time(cmd_matrix[curr_line][1] - 1);
      else calculate_total_time(curr_line + 1);
      break;
    default:
      break;
  }
}




// Fill out goto extra slot and setup temperature for cycles
void setup_gotos() {
  for (int i = 0; i < N_lines; i++) {
    if (cmd_matrix[i][0] == 1) cmd_matrix[i][3] = cmd_matrix[i][2];
  }
}





// Setup temperatures for each goto cycle
void setup_temp() {
  for (int i = 0; i < N_lines; i++) {
    if (cmd_matrix[i][0] == 0) cmd_matrix[i][1] -= cmd_matrix[i][3];
  }
}




// Calculates current goto of pcr program
int calculate_curr_goto() {
  for (int i = 0; i < N_lines; i++) if (cmd_matrix[i][0] == 1 && i > curr_goto) return i;
  return -1;
}





// Loop for display options and data presentation
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

      if (flag == 2) {
        lcd.setCursor(0, 1);
        lcd.print("TH:        C");
      }


      if (temp3 > 99.9) sprintf(aux_str, ">100\0");
      else if (temp3 < -99.9) sprintf(aux_str, "<-100\0");
      else dtostrf(temp3, 5, 1, aux_str);

      lcd.setCursor(3, 1);
      lcd.print(aux_str);
    } else {
      meantime = millis();
    }
  }
}




// Function that saves the pcr program on the Arduino memory
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






// Function that loads the pcr program from the Arduino memory
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


// Function that only works to troubleshoot and debug problems with this file
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
