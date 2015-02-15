/*This is my geocache box code. I used ladyada's tutorial on how to wire it, along
with the lcd display, and am using her library. This code is based on her example sketches for
the gps unit, lcd, along with other people's code and forum conversations on the topic

WHERE I AM SO FAR:
I have the distance calculator, destination calculator, attempts working, and check date function working.

I need to start adding the hardware modules (locking mechanism) and building the box.
*/

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <math.h>                     //I need the cosine function, so I need to include math
#include <EEPROM.h>

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);          // If using hardware serial (e.g. Arduino Mega), enable this line instead:
//Adafruit_GPS GPS(&Serial1);

/**********************
DEBUGGING
**********************/
const int serialDebug = 0;            //serial debugging. 1 for active, 0 for off
const int lcdDebug = 0;               //lcd debugging. 1 for active, 0 for off
#define GPSECHO  false                //echo GPS data to serial consol
int resetCounter = false;             //this resets the number of tries you have until the box opens itself. true resets the counter to 1 (for first attempt)

//variable
int firstOn = 0;                         //first time I turned this thing on, I want this to be true

//GPS stuff
boolean usingInterrupt = false;         //we're not using the interrupt
void useInterrupt(boolean);             // Func prototype keeps Arduino 0023 happy

//lcd library too
#include <LiquidCrystal.h>
LiquidCrystal lcd(7, 8, 6, 10, 11, 12);        // initialize the library with the numbers of the interface pins

//EEPROM
int address = 0;                           //the number of attempts is stored in address 0 in EEPROM

/**********
//Constants
***********/

//Destination
//I'm doing all of my calculations with the latitude and longitude in degrees, since that's what the haversine formula needs,
//therefore, I need to convert my coordinates into degrees in order to determine where I am using this formula.
//You have to put these two numbers into the function directly
//these have 6-7 decimal points of accuracy. Therefore, my code should account for that



//Santa Monica Pier, 25 miles from house
const float destinationLat = 34.007426;           //latitude of destination (Santa Monica Pier) in DEGREES
const float destinationLong = 118.499823;         //positive longitude of Santa Monica Pier in DEGREES
const float NemaDestinationLat = 3400.4456;      //NEMA sentance latitude for Santa Monica Pier
const float NemaDestinationLong = 11829.9894;    //NEMA sentance longitude for Santa Monica Pier
//NEMA sentence is degrees (2 decimals Lat, 3 decimals Long), minutes (two decimals.four decimals)



/*
//Luxor, Las Vegas, 36.095511,-115.176033, 212.5 miles from house
const float destinationLat = 36.095511;           //latitude of destination in DEGREES
const float destinationLong = 115.176033;         //positive longitude in DEGREES
const float NemaDestinationLat = 3605.73066;      //NEMA sentance latitude
const float NemaDestinationLong = 11510.56198;    //NEMA sentance longitude
*/
/*
After comparing the distance between my NEMA converted distance and a location, I was .6912 miles off. I'm going to give myself a mile
just to be safe. I can also check the location directly, which was accurate, but did not give me good distance measurements
*/
const float maxMilesAway = 1;                   // Max miles away for box to open (1760 yards). This is for calculating distance away. 
                                                //This is less accurate than comparing raw NEMA sentance output because of float limitations
const float maxGPSAway = .1;                    //this is .1 minutes away (from NEMA sentence). 1 minutes is 1 nautical mile, so this is .115 miles, or 200 yards 
const int dlay = 2000;                          //longest delay during lcd debug

                                          
const long waitTime = 300000;                     //longest I'll wait for a fix before timeing out and turing off (ms)

const int maxAttempts = 50;                      //maximum number of attempts to open the box
const int offPin = 4;                           //digital pin to turn off the device
//note: digital pin 13 fluctuates during power on. I can't really use this pin.

//Servo pins
const int servoPin = 9;                            //servo on pin 9

//Other destinations:
/* latitude, longitude
42.999155,-71.47135                              //island in the middle of a river in Manchester, NH. Approximately 3,000 miles away from house
34.160331, -118.131934                           //Roscoe's House of Chicken and Waffles, 1.76 miles away from house
34.007426, -118.499823                           //Stanta Monica Pier, in degrees 25 miles away
36.095511,-115.176033                            //The Luxor, Las Vegas

const float destinationLat = 34.007426;          //latitude of destination (Santa Monica Pier) in DEGREES
const float destinationLong = 118.499823;       //positive longitude of Santa Monica Pier in DEGREES
const float NemaDestinationLat = 3400.45138      //NEMA sentance latitude for Santa Monica Pier
const float NemaDestinationLong = 11829.99178    //NEMA sentance longitude for Santa Monica Pier
*/

void setup()  
{
  if (serialDebug == 1){
  Serial.begin(115200);                                 // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.println("Grant's GPS Present Test");
  }
  lcd.begin(16, 2);                                     //begin lcd

  GPS.begin(9600);                                      //GPS uses 9600 baud
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);         //RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);            // 1 Hz update rate
  useInterrupt(true);                                   //see if using interrups
  
  //set pins to output/input
  pinMode(offPin, OUTPUT);
  pinMode(servoPin, OUTPUT);
  digitalWrite(servoPin, LOW);
  
  //servo stuff
  //myservo.attach(servoPin);
  
  
}



SIGNAL(TIMER0_COMPA_vect) {       // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
  char c = GPS.read();
  if (GPSECHO)                   // if you want to debug, this is a good time to do it!
    if (c) UDR0 = c;             // writing direct to UDR0 is much much faster than Serial.print but only one character can be written at a time.
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;            // Timer0 is already used for millis() - we'll just interrupt somewhere in the middle and call the "Compare A" function above
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);   // do not call the interrupt function COMPA anymore
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

/*******
LOOP DA LOOP!
*******/
void loop()
{
  //I'm using the interrupt, so I don't need the not using interrupt part here
  if (GPS.newNMEAreceived()) { // if a sentence is received, we can check the checksum, parse it...
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  if (timer > millis())  timer = millis();              // if millis() or timer wraps around, we'll just reset it
  
  if (resetCounter){                                   //if I need to reset the value in EEPROM
    EEPROM.write(address, 1);                          //set it to 1, so when you turn it on the next time, it's the first attempt
    resetCounter = false;                              //don't reset it again though
  }
  
  if (millis() - timer > 2000) {                       // approximately every 2 seconds or so, check stuff
    timer = millis(); // reset the timer
    
    //now, I want to check to see whether or not I have a fix. If not, I want to run a piece of code
    //that waits for a fix and turns off if I don't get one soon enough
    
    if (GPS.fix){
      if(lcdDebug){         //I want to know how long it took to get a fix, so I get an idea of what's going on
        lcd.clear();
        lcd.home();
        lcd.print("It took (s)");
        lcd.setCursor(0,1);
        long seconds = timer/1000;
        lcd.print(seconds);
        delay(dlay);       //wait until I can read it
      }
      //my checkDistance and checkDate functions return a distance (float) or a number of days (int). 0 means it's good, a number means I 
      //need to print something, because I'm not there yet
      float myDistance = checkDistance();                   //gives me my distance from destination
      int myDate =  checkDate();                             //gives me the date from target date
      
      
      if (myDistance == 0.0 && myDate == 0){
        openBox();     //if I'm there, and it's time, open!
      }
      else{           //if not, then print the clues
        lcd.clear();
        lcd.home();
        lcd.print(myDistance);
        lcd.setCursor(0,1);
        lcd.print("Miles");
        delay(5000);
        lcd.clear();
        lcd.home();
        lcd.print(myDate);
        lcd.setCursor(0,1);
        lcd.print("Days");
        delay(5000);
        turnOff(1);                                      //turn off and increment the attempts 
      }
    }
    else{
      initialization();                 //this runs until a fix is found or the unit powers down
    }
  }
 
}


/**************
//this code waits for a fix and turns off if I doesn't get one soon enough
**************/
void initialization( void ){
  //the first thing that I need to do is print out stuff for Lily to read, then I need to wait for a fix
  //and if I don't get one soon enough, just power off.
  //I can use timer, which is already an unsigned long
  timer = millis();
  if (firstOn == 0) {             
  
  //if this is the first time I've turned on, then I need to say hello
  //first, I'm going to get how many attempts she's made
  byte attempts = EEPROM.read(address);
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("Hello Lily");
  lcd.home();
  lcd.print("Attempt ");
  lcd.print(attempts, DEC);
  int attemptsLeft = maxAttempts - attempts;
  delay(5000);
  lcd.setCursor(0,1);
  lcd.print(attemptsLeft);
  lcd.print(" attempts left");
  lcd.home();
  lcd.print("Go outside");
  delay(5000);
  firstOn = 1;
  }
  
  if (firstOn == 1){           //if not, I'm calculating
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("Finding signal");
  lcd.home();
  lcd.print("Please wait");
  firstOn++;
  }
  
  if (timer > waitTime) turnOff(0);       //if I've waited to long, stop wasting battery and don't count it
  
  //once I have a gps fix, then I can actually do the calculations. I've already got the lcd
  //saying what I want, so go ahead and calculate stuff
}
/*********
Turns off everything
if it gets a 1, it turns off and increments the attempts. If it gets a 0, it turns off without incrementing attempts
if it gets anything else (like a 2), it will just say powering off and turn off
**********/
void turnOff(int attemptPlus){
  //this will turn off my arduino. I don't know how this will happen yet, but it will happen for now,
  //I'll just have it say that it's powering off
  
  if (attemptPlus == 1){                           //if I got a signal and am not in the right place, count the attempt
    byte attempts = EEPROM.read(address);
    attempts += attempts;
    EEPROM.write(address, attempts);
  }
  
  if (attemptPlus == 0){
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("No signal");
    delay(5000);
  }
  
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("Powering off");
  delay(5000);
  
  //turn myself off with pololu switch
  digitalWrite(offPin, HIGH);
  
  //if I don't turn off, then I'm externally powered, and I want to open the box
  delay(50);
  openBox();
 
}

/********
Opens Box
********/

void openBox(void){
 //this will open the box. I'll write this once I have motors wired up. 
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("Opening Box");
  digitalWrite(servoPin, HIGH);
  delay(5000);
  turnOff(2);                       //2 just makes it turn off
  
}

/*************************************************************************
   //Function to check the distance from destination
   //this returns 0 for there, or the distance away in miles
 *************************************************************************/

float checkDistance( void ){
  //I'm going to calculate the distance that I have between me and the target location using the haversine
  //function. I got the function from the arduino forums
  float gpsLat = NemaToDegrees(GPS.latitude);
  float gpsLong = NemaToDegrees(GPS.longitude);
  if (lcdDebug){
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("Latitude");
  lcd.home();
  lcd.print(gpsLat,6);
  delay(dlay);
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("Longitude");
  lcd.home();
  lcd.print(gpsLong,6);
  delay(dlay);
  }
  
  //I'm going to find the approximate distance and save it to miles away
  float milesAway = calcdist( destinationLat , destinationLong , gpsLat , gpsLong);
  
  //here I'm comparing the raw NEMA sentance instead of calculating the distance. This is more accurate, but does not
  //give me how far I am away
  float latCheck = abs(NemaDestinationLat - GPS.latitude);
  float longCheck = abs(NemaDestinationLong - GPS.longitude);
  
  if (lcdDebug){
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("Distance");
    lcd.home();
    lcd.print(milesAway,6);
    delay(dlay);
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("LatCheck");
    lcd.home();
    lcd.print(latCheck,6);
    delay(dlay);
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("LongCheck");
    lcd.home();
    lcd.print(longCheck,6);
    delay(dlay); 
  }
  
  
  if (latCheck < maxGPSAway && longCheck < maxGPSAway) return 0.0;           //this checks the raw coordinates to see if I'm close enough
  
  //now I'm going to check to see if I'm close enought to my destination using the distance
  if (milesAway < maxMilesAway) return 0.0;                                 //checks my distance to see if I'm close enough
  else{
    //if I'm not there, return how far away I am
  return milesAway;
  }
}

/******************
//I'm calling these functions only if I have a fix
//this is my function to check to see if I'm on the correct day
//this returns 0 for I'm at/past the correct day (it can open after her birthday)
//and returns the number of days to go if not
*******************/
int checkDate( void ) {
  int gpsMonth = GPS.month;
  int gpsDay = GPS.day;
  //so, it looks like the GPS unit is on June 16, 2012 at 5:35 when I'm at June 15, at 10:37 pm
  //so it's either 7 hours ahead of me, or it's just a day ahead and the time is off.
  //I'm going to ignore this difference, and let her open the box early. Oh darn.
  if (lcdDebug){
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("Month");
  lcd.home();
  lcd.print(gpsMonth);
  delay(dlay);
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("Day");
  lcd.home();
  lcd.print(gpsDay);
  delay(dlay);
  }
  
  
  if (gpsMonth > 9){       //October is month 10
  //however, I'm one day ahead with this gps unit
    return 0;
  }
  else{                       //otherwise, I'm not there
    //I need to calculate how many days I have to go
    int daysLeft = 0;
    //I'm giving this to her in August, so I just need to program it to count down from August to October
    if (gpsMonth == 8){
      if (serialDebug) Serial.println("August");
      //I'm in August. I have 30 days in September, then the rest of August (31 days)
      daysLeft = 30 + 31 - gpsDay + 1;         //this should be the days left before October
    }
    else if ( gpsMonth == 7){
      if (serialDebug) Serial.println("July");
      daysLeft = 31 + 31 +30 - gpsDay + 1; 
    }      //June = 30 days, July = 31 days, August = 31 days, September = 30 days 
    else{
      if (serialDebug) Serial.println("September");
     daysLeft = 30 - gpsDay + 1;                //if it's not August, it's September, so subtract the day I'm at 
    }
    return daysLeft;
  }
}

/*************************************************************************
 * //Function to calculate the distance between two waypoints
 * //I got this from the arduino forums.
 *************************************************************************/
float calcdist(float flat1, float flon1, float flat2, float flon2)
{
float dist_calc=0;
float dist_calc2=0;
float diflat=0;
float diflon=0;

//I've to spplit all the calculation in several steps. If i try to do it in a single line the arduino will explode.
diflat=radians(flat2-flat1);
flat1=radians(flat1);
flat2=radians(flat2);
diflon=radians((flon2)-(flon1));

dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
dist_calc2= cos(flat1);
dist_calc2*=cos(flat2);
dist_calc2*=sin(diflon/2.0);
dist_calc2*=sin(diflon/2.0);
dist_calc +=dist_calc2;

dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

dist_calc*=3958.76; //Converting to miles
//Serial.println(dist_calc);
return dist_calc;
}

/************
NEMA parser
Takes in the NEMA sentance values (degree.minutes) and outputs degrees
This is off by .3 miles (per calculation) due to the constraints of floats. a.k.a, I calculated it once, and I was off by 
.3 miles. It may be more for other destinations.
*************/

float NemaToDegrees( float NemaValue ){
  int NemaDegrees = NemaValue/100;                                 //this gives me degrees
  if (serialDebug){
    Serial.println("Nema degrees Calculated");
    Serial.println(NemaDegrees);
  }
  float NemaMinutes = (NemaValue - (float)NemaDegrees*100.0);      //this gives me the minutes (positive value, since NemaValue is larger
  if (serialDebug){
    Serial.println("Nema Minutes Calculated");
    Serial.println(NemaMinutes,4);
  }
  return (float)NemaDegrees + (NemaMinutes/60);                     //convert Minutes to degrees, add them, and return value
}
