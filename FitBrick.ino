/*********************************************************************************
 * 
 *  This is the FitBrick code used for the FitBrick assembled for BME 354 project.
 *  The same hardware and functionality mentioned in the lab report was used 
 *  with the addition of a switch for battery power.
 * 
 * Conventions used in code:
 * small letters and '_' for variable names (e.g. footfalls, lifetime_footfalls)
 * capital initial for method names (e.g. displayFootfalls)
 * all capitals and '_' for defines and constants (e.g. THRESHOLD, ACTIVITY_SAMPLES)
 * 
 * April 25, 2016
 * @authors Saeed Alrahma, Eric Musselman, Varun Gudapati
 * 
 * Credit to @author K. Townsend (Adafruit Industries) for @code MMA8451demo
 * demonstrating how to usethe MMA8451 accelerometer
 * Credit to @author (Adafruit Industries)  for @code HelloWorld
 * demonstrating how to use the RGBLCDShield
 *
 **********************************************************************************/

 /* LIBRARIES */
#include <Wire.h>
// LCD Shield libraries
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
// Accelerometer librarires
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
// EEPROM
#include <EEPROM.h>

/* DEFINES and CONSTANTS*/
// LCD backlight colors
#define WHITE_LOW 0x7
#define YELLOW_MODERATE 0x3
#define RED_INTENSE 0x1
#define OFF 0x0
#define FOOTFALL_MODE 1
#define MOMENT_MODE 2
#define DISTANCE_MODE 3
#define APNEA_MODE 4
#define CALORIES_MODE 5
#define TIMER_MODE 6
#define USER_CALIBRATION_MODE 7
// Samples for activity intensity calculations
const unsigned short ACTIVITY_SAMPLES = 20; // number of samples to measure activity intensitty
const short NUM_MODES = 7;  // change if number of display modes changes
const byte WRITE_FOOTFALLS_THRESHOLD = 10; // write to eeprom every 10 steps
const float APNEA_THRESHOLD = 9.7; // acceleration threshold for apnea
const unsigned short APNEA_TIME = 5000; // time in ms for apnea detection

/* GLOBAL VARIABLES */
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield(); // LCD variable
Adafruit_MMA8451 mma = Adafruit_MMA8451(); // accelerometer (MMA) variable
unsigned long lifetime_footfalls = 0; // lifetime footfalls
unsigned long resettable_footfalls = 0; // resettable footfalls
String activity_level = "LOW"; // activity level
byte activity_history_index = 0; // index of current footfall in the history (array loops around)
byte display_mode = 1; // current display mode (numbered in "#define" above)
boolean activity_history[ACTIVITY_SAMPLES]; // array to track activity history for activity intensity
boolean bright = true; // track LCD backlight (false = off)
boolean onepass = false; // track accelerometer data for footfall sensing
float previous_magnitude = 0; // track accelerometer magnitude for footfall sensing
boolean mode_switched = true; // track when display mode is switched
float distance = 0; // track distance traveled based on footfalls and user input stride length
float stride_length = 1.5; // default value 1.5, resettable by user
unsigned short eeprom_address = 0; // address of lifetime footfalls in EEPROM
unsigned long eeprom_write_count = 0; // current EEPROM block writes count
byte write_footfalls_count = 0; // write to EEPROM every time count reaches threshold
unsigned long apnea_start_time = 0; // save start time for apnea measurements
boolean apnea_detected = false; // track when apnea is detected
unsigned long timer_start = 0; // track timer start time
unsigned long timer_time = 0; // track current timer time
boolean timer_stop = true; // track when timer is start/stop
float threshold = 11.5; // acceleration threshold for footfall
float max_magnitude = 0; //Max magnitude of current step
float overall_max_magnitude = 0; //Min of all maxes
int calib_count = 0; //Number of steps analyzed


/** Setup LCD and Accelerometer (MMA).
 *  Load lifetime footfalls from EEPROM
 */
void setup() {
  Serial.begin(9600);
  /* setup LCD */
  lcd.begin(16, 2); // setup # of columns and rows
  lcd.setBacklight(WHITE_LOW); // set background color to white

  /* setup accelerometer (MMA) */
  // error: MMA failed
  if (!mma.begin()) {
    printTwoLines("Error", "MMA failed...");
    while (1);
  }
  // MMA found
  printTwoLines("   WELCOME TO", "    FitBrick");
  mma.setRange(MMA8451_RANGE_2_G);
  delay(500);//*********************** add delay if loading from eeprom not long enough to display welcome ************

  /* load lifetime footfalls from EEPROM */
  loadLifetimeFootfalls();

  /* calibrate and display threshold */
  autoCalibrate();
  lcd.clear();
  lcd.print("Threshold (m/s^2)");
  lcd.setCursor(0, 1);
  lcd.print(threshold);
  delay(1000);
  
  /* Display initial screen */
  lcd.clear();
  printFootfalls();

}

/** check for button clicks
 *  sense footfalls
 *  display current mode
 */
void loop() {
  // track footfalls to check for change
  unsigned long previous_lifetime_footfalls = lifetime_footfalls;
  unsigned long previous_resettable_footfalls = resettable_footfalls;
  senseFootfall(); // check for footfall
  
  // check and implement button clicks
  uint8_t buttons = lcd.readButtons();
  if (buttons) {
    // RIGHT: toggle screen on/off
    if (buttons & BUTTON_RIGHT) {
      toggleDisplayBacklight();
    }
    // LEFT: switch mode
    if (buttons & BUTTON_LEFT) {
      display_mode++; // Move to next mode
      mode_switched = true;
      apnea_detected = false; // restart apnea detection
      if (display_mode > NUM_MODES) { // wrap around
        display_mode = 1;
      }
    }
  }
  
  // only display when screen is bright
  // or in apnea mode
  if(bright || display_mode == APNEA_MODE) {
    // display current mode
    switch (display_mode) {
      case FOOTFALL_MODE:
        displayFootfalls(previous_lifetime_footfalls);
        break;
      case MOMENT_MODE:
       displayMoment();
       break;
      case DISTANCE_MODE:
        displayDistance(previous_lifetime_footfalls);
        break;
     case APNEA_MODE:
        if(!apnea_detected) {
          displayApnea();
       }
        break;
     case CALORIES_MODE:
        displayCalories(previous_lifetime_footfalls, previous_resettable_footfalls);
        break;
     case TIMER_MODE:
        displayTimer();
        break;
     case USER_CALIBRATION_MODE:
        displayUserCalibration(previous_resettable_footfalls);
        break;
     default: displayFootfalls(previous_lifetime_footfalls);
    }
  } else {
    senseFootfall(); // only sense movement
  }
  

}

/** check accelerometer (MMA) for footfall occurences.
 *  add +1 step (lifetime and resettable) if footfall measured
 *  @return true if footfall occured, false otherwise
 */
boolean senseFootfall() {
  boolean footfall = false;
  sensors_event_t event = getAcceleration(); // read acceleration from MMA

  // calculate magnitude of acceleration
  float magnitude = sqrt(event.acceleration.x*event.acceleration.x + event.acceleration.y*event.acceleration.y 
  + event.acceleration.z*event.acceleration.z);

  // check for footfall
  if(onepass = false && previous_magnitude != 0 && magnitude > threshold && previous_magnitude < threshold)
  {
    onepass = true;
  }
  if(onepass = true)
  {
    if(magnitude < threshold && previous_magnitude > threshold)
    {
      onepass = false;
      // add +1 step to lifetime and resettable counts
      lifetime_footfalls++;
      resettable_footfalls++;
      write_footfalls_count++;
      if(write_footfalls_count>=WRITE_FOOTFALLS_THRESHOLD) {
        writeLifetimeFootfalls(); // write to EEPROM
        write_footfalls_count=0; // reset count
      }
      updateActivityHistory(1); // history for activity intensity
      footfall = true;
    } else {
      updateActivityHistory(0); // history for activity intensity
    }
  }
  previous_magnitude = magnitude;
  delay(100); // *************POSSIBLY REMOVE or MODIFY*************
  return footfall;
}

/** updates the activity array with most recent value
  * @param footfall holds 0 or 1 for footfall at current sample
*/
void updateActivityHistory(boolean footfall) {
  if(activity_history_index>ACTIVITY_SAMPLES) {
    activity_history_index = 0;
  }
  activity_history[activity_history_index] = footfall;
  activity_history_index++;
}

/** update activity level (global variable)
 *  based on # of steps in a defined time
 *  LOW < 20% <= MODERATE <= 35% < INTENSE
 *  Percentage refers to percentage of steps sensed
 *  from a fixed number of footfall sensing trials
*/
void checkActivityLevel() {
  float avg = 0;
  
  // Get the average of the most recent saved activity
  for(int i=0; i<ACTIVITY_SAMPLES; i++) {
    avg += activity_history[i];
  }
  avg = avg/ACTIVITY_SAMPLES;
  
  // Change activity level and backlight based on the average
  if(avg<0.2) { // LOW
    activity_level = "LOW";
  } else if (avg>0.35) { // INTENSE
    activity_level = "INTENSE";
  } else { // MODERATE
    activity_level = "MODERATE";
  }
}

/** display total lifetime footfalls, 
  * current footfall count, 
  * display level of activity
*/
void displayFootfalls(unsigned long previous_lifetime_footfalls) {
  displayModeName("Lifetime", "Footfalls");

  // update LCD only if footfall changed
  if (lifetime_footfalls > previous_lifetime_footfalls) {
    checkActivityLevel(); // update activity level
    printFootfalls();
    setBacklightColor();
  }

  // check and implement button clicks
  uint8_t buttons = lcd.readButtons();
  if (buttons) {
    // SELECT: reset footfalls
    if (buttons & BUTTON_SELECT) {
      resettable_footfalls = 0; //
      printFootfalls(); // refresh display
    }
  }
  

}

/** displays moment-to-moment x, y, z accelerations about twice a second
 */
void displayMoment() {
  displayModeName("M-to-M", "Acceleration");  // display mode name
  
  sensors_event_t event = getAcceleration(); // read acceleration from MMA

  // save x, y, and z-accelerations in easy to access variables
  float xa = event.acceleration.x;
  float ya = event.acceleration.y;
  float za = event.acceleration.z;

  // Display the results (acceleration is measured in m/s^2)
  // Display x acceleration
  lcd.setCursor(0, 0);
  lcd.print("X:");
  lcd.setCursor(2, 0);
  if (xa > 0) {
    if (xa < 10) {
      lcd.print(xa, 3);
    }
    else if (za >= 100) {
      lcd.print(xa, 1);
    }
    else {
      lcd.print(xa, 2);
    }
  }
  else {
    if (xa > -10) {
      lcd.print(xa, 2);
    }
    else if (za <= -100) {
      lcd.print(xa, 0);
    }
    else {
      lcd.print(xa, 1);
    }
  }

  // Display y acceleration
  lcd.setCursor(8, 0);
  lcd.print("Y:");
  lcd.setCursor(10, 0);
  if (ya > 0) {
    if (ya < 10) {
      lcd.print(ya, 3);
    }
    else if (ya >= 100) {
      lcd.print(ya, 1);
    }
    else {
      lcd.print(ya, 2);
    }
  }
  else {
    if (ya > -10) {
      lcd.print(ya, 2);
    }
    else if (ya <= -100) {
      lcd.print(ya, 0);
    }
    else {
      lcd.print(ya, 1);
    }
  }

  // Display z acceleration
  lcd.setCursor(0, 1);
  lcd.print("Z:");
  lcd.setCursor(2, 1);
  if (za > 0) {
    if (za < 10) {
      lcd.print(za, 3);
    }
    else if (za >= 100) {
      lcd.print(za, 1);
    }
    else {
      lcd.print(za, 2);
    }
  }
  else {
    if (za > -10) {
      lcd.print(za, 2);
    }
    else if (za <= -100) {
      lcd.print(za, 0);
    }
    else {
      lcd.print(za, 1);
    }
  }
  
  delay(400); // display acceleration about twice per second
}

/** display distance mode
 *  take user input stride length
 *  measure distance through sensing footfalls
 */
void displayDistance(unsigned long previous_lifetime_footfalls) {
  displayModeName("Distance Mode", ""); // display mode name
  
  if(previous_lifetime_footfalls<lifetime_footfalls) {
    distance = distance + stride_length;
    printDistance(); // update display
  }

  // check and implement button click
  uint8_t buttons = lcd.readButtons(); // check for button click
  if (buttons) {
    // UP increases the stride lengthe=
    if (buttons & BUTTON_UP) {
      if(stride_length < 9) {
        stride_length = stride_length + 0.1;
      }
    }
    // DOWN lowers the stride length
    if (buttons & BUTTON_DOWN) {
      if(stride_length > 1) {
        stride_length = stride_length - 0.1;
      }
    }
    // SELECT resets the distance
    if (buttons & BUTTON_SELECT) {
      distance = 0;
      //fresh = true;
    }
    // refresh display
    lcd.clear();
    printDistance(); 
    delay(100);
  }
}

/** checks acceleration values due to breathing
 *  detects apnea if acceleration values
 *  do NOT pass breathing threshold
 *  prints directly to the display
 */
void displayApnea() {
  displayModeName("Apnea Detection", "");
  
  unsigned long current_time = millis();
  // if 5 seconds passed without breathing detected
  // then print apnea detected
  if(current_time-apnea_start_time > APNEA_TIME) {
    lcd.clear();
    lcd.println("Apnea Detected");
    lcd.setBacklight(RED_INTENSE);
    apnea_detected = true;
  }

  // continuously sense accelerometer movement for Apnea
  if(!senseApnea()){
    apnea_start_time = millis(); // restart time if normal breathing detected
  }
}

/** compares the acceleration magnitude
 *  to breathing magnitude
 * @return false if normal breathing detected, true otherwise
 */
boolean senseApnea(){
  sensors_event_t event = getAcceleration(); // read acceleration from MMA

  // calculate magnitude of acceleration
  float magnitude = sqrt(event.acceleration.x*event.acceleration.x + event.acceleration.y*event.acceleration.y 
  + event.acceleration.z*event.acceleration.z);

  // regular breathing (no apnea)
  if(magnitude > APNEA_THRESHOLD) {
    delay(100);
    return false;
  }
  // possible apnea
  delay(100);
  return true;
}

/** calculates and displays calories burned through footfalls 
 */
void displayCalories(unsigned long previous_lifetime_footfalls, unsigned long previous_resettable_footfalls) {
  displayModeName("Calories Burned", "");
  if(previous_lifetime_footfalls != lifetime_footfalls ||
     previous_resettable_footfalls != resettable_footfalls) {
      printCalories(); 
     }
}

/** calculates calories based on footfalls
 *  average formula is calories = footfalls/20
 */
unsigned long calculateCalories(unsigned long footfalls){
  return footfalls/20;
}

/** starts/stops a timer on button click
 * prints the timer on the display
 */
void displayTimer() {
  displayModeName("Timer", "");

  // check and implement button clicks
  uint8_t buttons = lcd.readButtons();
  if (buttons) {
    // UP: Start/Stop
    if (buttons & BUTTON_UP) {
      if(timer_stop) {
        timer_start = millis();
      } 
      timer_stop = !timer_stop;
    }
    
    // SELECT: reset timer
    if (buttons & BUTTON_SELECT) {
      timer_time = 0;
      timer_stop = true;
    }
  }

  // keep track of timer
  if(!timer_stop) {
    timer_time = (millis()-timer_start)/1000;
    // print timer
    printTimer();
  }
  delay(100);

}

/** convert the time from seconds
 *  to hr:min:sec format and
 *  prints it to display
 */
void printTimer(){
  if(timer_time%60==0) { // clear screen every minute
    lcd.clear();
  }
  lcd.setCursor(0, 0);
  lcd.print(timer_time/3600);
  lcd.print(":");
  lcd.print((timer_time/60)%60);
  lcd.print(":");
  lcd.print(timer_time%60);
}

/** allows user to adjust sensor threshold
 */
void displayUserCalibration(unsigned long previous_resettable_footfalls) {
  displayModeName("User Calibration", "");

  // check and implement button clicks
  uint8_t buttons = lcd.readButtons();
  if (buttons || previous_resettable_footfalls<resettable_footfalls) {
    lcd.clear();
    // Set threshold between 10 to 20
    // UP increases the threshold
    if (buttons & BUTTON_UP) {
      if(threshold < 20)
      {
        threshold = threshold + 0.1;
      }
    }
    // DOWN lowers the threshold
    if (buttons & BUTTON_DOWN) {
      if(threshold > 10)
      {
        threshold = threshold - 0.1;
      }
    }
    // update display
    printUserCalibration();
  }
}

/** prints current threshold value on display
 */
void printUserCalibration(){
  lcd.setCursor(0, 0);
  lcd.print("Thr: ");
  lcd.print(threshold, 1);
  lcd.print(" m/s^2");
  lcd.setCursor(0, 1);
  lcd.print("Steps: ");
  lcd.print(resettable_footfalls);
  delay(100); 
}

/** Toggles LCD backlight on and off to conserve battery
 */
void toggleDisplayBacklight(){
  if(bright) { // turn OFF
    lcd.setBacklight(OFF);
    delay(200);
  }
  else {  // turn ON
    printCurrentDisplay();
    setBacklightColor();
    
  }
  bright = !bright;  // toggle bright variale
  //delay(200);
}

/** sets the LCD backlight color based on activity intensity
 * White --> LOW
 * Yellow --> MODERATE
 * Red --> INTENSE
 */
short setBacklightColor(){
  if(activity_level == "INTENSE"){
    lcd.setBacklight(RED_INTENSE);
  } else if (activity_level == "MODERATE"){
    lcd.setBacklight(YELLOW_MODERATE);
  } else { // activity_level == "LOW" (default)
    lcd.setBacklight(WHITE_LOW);
  }
}

/** Load lifetime footfalls from EEPROM
 *  and store value in global variable
 *  7-byte block
 *  byte 0: LSBs of number of writes
 *  byte 1: middle bits of number of writes
 *  byte 2: MSBs of number of writes
 *  total writes = 2^16 = 65,536 writes
 *  byte 3: LSBs of lifetime footfalls
 *  byte 4 and 5: middle bits of lifetime footfalls
 *  byte 6: MSBs of lifetime footfalls
 *  max lifetime footfalls = 2^32 = ~4.3 billions
 */
void loadLifetimeFootfalls(){
  // skip worn out (old) blocks of bytes
  eeprom_write_count = readBytesFromEEPROM(3, eeprom_address);
  while(eeprom_write_count>90000){
    eeprom_address += 7;
    eeprom_write_count = readBytesFromEEPROM(3, eeprom_address);
    }
  
  // read lifetime footfalls from EEPROM
  lifetime_footfalls = readBytesFromEEPROM(4, eeprom_address+3);
}

/** Save lifetime footfalls to EEPROM
 *  in 7-byte block
 *  byte 0: LSBs of number of writes
 *  byte 1: middle bits of number of writes
 *  byte 2: MSBs of number of writes
 *  total writes = 2^16 = 65,536 writes
 *  byte 3: LSBs of lifetime footfalls
 *  byte 4 and 5: middle bits of lifetime footfalls
 *  byte 6: MSBs of lifetime footfalls
 *  max lifetime footfalls = 2^32 = ~4.3 billions
 */
void writeLifetimeFootfalls(){
  eeprom_write_count++; // increment number of writes
  
  // discard block if it wore out and move to next
  if(eeprom_write_count>90000) {
    // discard block of bytes
    for(int i=0; i<3; i++){
      EEPROM.write(eeprom_address+i, 255);
    }
    eeprom_address += 7; // move to next block
    eeprom_write_count = 0; // reset number of writes
  }

  // update number of writes in eeprom
  writeBytesToEEPROM(3, eeprom_address, eeprom_write_count);
  
  // update lifetime footfalls in eeprom
  writeBytesToEEPROM(4, eeprom_address+3, lifetime_footfalls);
}

/** read acceleration from accelerometer (MMA)
 *  @return Accelerometer (MMA) event with (x, y, z) acceleration raw values
 */
sensors_event_t getAcceleration(){ 
  // Get a new sensor event 
  sensors_event_t event; 
  mma.getEvent(&event);

  return event;
}

/** print current footfalls and activity data on display
 */
void printFootfalls() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(resettable_footfalls);
  lcd.print(" (");
  lcd.print(activity_level);
  lcd.print(")");
  lcd.setCursor(0, 1);
  lcd.print(lifetime_footfalls);
}

/** prints current stride_length and distance values to display
 */
 void printDistance() {
  lcd.setCursor(0, 0);
  lcd.print("Stride: ");
  lcd.print(stride_length, 1);
  lcd.print(" ft");
  lcd.setCursor(0, 1);
  lcd.print("Dist: ");
  if(distance>5280) { // reached a mile
    if((distance/5280)<10) {
      lcd.clear(); // clear lcd when switching to miles
    }
    lcd.print(distance/5280, 1);
    lcd.print(" mi");
  } else {
    lcd.print(distance, 1);
    lcd.print(" ft");
  }
 }

 /** prints calories burned and current
  *  activity steps on display
 */
 void printCalories() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Activity: ");
  lcd.print(calculateCalories(resettable_footfalls), 1);
  lcd.setCursor(0, 1);
  lcd.print("Life: ");
  lcd.print(calculateCalories(lifetime_footfalls), 1);
 }

/** prints two lines to the LCD
 *  @param line1 String to print on line 1 of LCD
 *  @param line2 String to print on line 2 of LCD
 */
void printTwoLines(String line1, String line2){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

/** displays the mode name when mode is switched
 *  @param line1 String to print on row 1 of LCD
 *  @param line2 String to print on row 2 of LCD
 */
void displayModeName(String line1, String line2){
  if(mode_switched) { // only if display mode was switched
    printTwoLines(line1, line2);
    mode_switched = false;
    delay(500); // keep mode name printed for half a second

    // display current mode before any measurements
    printCurrentDisplay();
  }
}

/** displays current mode screen
 *  before making any measurements/changes
 *  only for modes that do NOT always refresh display
 */
void printCurrentDisplay(){
    lcd.clear();
    switch(display_mode){
      case FOOTFALL_MODE:
        printFootfalls();
        break;
      case MOMENT_MODE:
        lcd.clear();
        break;
      case DISTANCE_MODE:
        printDistance();
        break;
      case APNEA_MODE:
        printTwoLines("Normal breath", "No apnea");
        apnea_start_time = millis(); // start apnea timing
        break;
      case CALORIES_MODE:
        printCalories();
        break;
      case TIMER_MODE:
        printTimer();
        break;
      case USER_CALIBRATION_MODE:
        printUserCalibration();
        break;
      default: // FOOTFALL_MODE
        printFootfalls();
        break;
    }
}

/** shift and mod number to get 1 byte a time
 *  writes 1 byte at a time to EEPROM
 *  @param num_bytes number of bytes to write
 *  @param start_address address of first byte to write
 *  @param number the long-type (4-byte) number to write
 */
void writeBytesToEEPROM(int num_bytes, int start_address, unsigned long number) {
  for (int i=0; i<num_bytes; i++ ){
    byte one_byte = ((number>>8*i)%256);
    EEPROM.write(start_address+i, one_byte);
  }
}

/** read 1 byte at a time from EEPROM
 *  shift and add bytes to get long-type number
 *  @param num_bytes number of bytes to read
 *  @param start_address address of first byte to read
 *  @return the long-type (4-byte) number read from EEPROM
 */
unsigned long readBytesFromEEPROM(int num_bytes, int start_address) {
  unsigned long number = 0;
  for (int i=0; i<num_bytes; i++){
    number += ((EEPROM.read(start_address+i))<<(8*i));
  }
  return number;
}

/** calibrates the accelerometer threshold at startup
 * Uses data from 9 steps and calculates threshold
 * from average value
 */
void autoCalibrate() {
  printTwoLines("walk until", "calibration over");
  
  while(calib_count < 9) {
    sensors_event_t event = getAcceleration(); // get acceleration data

    float magnitude = sqrt(event.acceleration.x*event.acceleration.x + event.acceleration.y*event.acceleration.y 
    + event.acceleration.z*event.acceleration.z); // calculate acceleration magnitude

    // check magnitude vs max
    if(magnitude > max_magnitude) {
      max_magnitude = magnitude; //adjust peak values
    }
    // Passed peak/stable threshold
    if(magnitude < 9.8 && previous_magnitude > 9.8) {
      // update overall max value
      if (overall_max_magnitude == 0) {
        overall_max_magnitude = max_magnitude; // first max
      }
      overall_max_magnitude = (overall_max_magnitude+max_magnitude)/2; // average max values
      max_magnitude = 0; // reset to 0 for next trial
      calib_count++; // add count only if passed peak/stable threshold
    }
    previous_magnitude = magnitude; // used in next trial
    delay(100);
  }
  //Set threshold based on minimum peak of walking
  threshold = overall_max_magnitude*0.9; 
}
