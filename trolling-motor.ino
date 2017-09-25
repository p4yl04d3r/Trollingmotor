#include <NewSoftSerial.h>
#include <TinyGPS.h>  
#include <Wire.h>
#define speedesc 125
// tiny gps library code
TinyGPS gps;
NewSoftSerial nss(3, 2);    // rx(3) and tx(2) pins in use
const int buttonPin = 9;      // the number of the pushbutton pin
const int motorPin = 10;      // the number of the motor relay pin
const int rightPin = 11;      // the number of the right relay pin
const int leftPin = 12;       // the number of the left relay pin
const int ledPin =  13;       // the number of the LED pin
int HMC6352Address = 0x42;
int slaveAddress;
float DISTANCE; //Distance returned by distance function
float BEARING;  //BEARING returned by bearing function
float flat;                 //GPS current lat
float flon;                 //GPS current lon
float save_flat=0;          //GPS saved lat
float save_flon=0;          //GPS saved lon
int buttonState = 0;        //Initial state of the button

void gpsdump(TinyGPS &gps);
bool feedgps();
void printFloat(double f, int digits = 2);

void setup()
{
  // Setup Compass
  slaveAddress = HMC6352Address >> 1;  
  Wire.begin();
  Serial.begin(115200);
  nss.begin(4800);
  // Setup output pins for driving
  pinMode(motorPin, OUTPUT);  //Motor relay 10
  pinMode(rightPin, OUTPUT);  //Right relay 11
  pinMode(leftPin, OUTPUT);   //Left relay 12
  pinMode(ledPin, OUTPUT);    //Button on/off LED 13
  //Setup input pins for Button 
  pinMode(buttonPin, INPUT);  //Butten signals to save current flat flon from GPS 9.

  delay(5000);
}

void loop()
{ 
  bool newdata = false;
  unsigned long start = millis();
  while (millis() - start < 250)            // get new data every 1/4 of a second
  {
    if (feedgps())
      newdata = true;
  }

  if (newdata) // If GPS has lock and data.
  {
//    Serial.println("Acquired Data");
//    Serial.println("-------------");
    //    gpsdump(gps);                      //Use to debug GPS data only.
    getlocation(gps);                        //Store flat and flon. Always runs.
    if (check_button())   {                        //If button is ON we want to go live
      save_location();                       //Store flat and flon.
      DISTANCE=distance();                   //Get distance to stored waypoint and current location
      BEARING=bearing();                     //Get bearing to stored waypoint from current location
      Serial.print("Distance: "); printFloat(DISTANCE,5);
      Serial.println();
      Serial.print("Bearing to waypoint: "); printFloat(BEARING,5);
      /* 
       Add Function to point compass at bearing to target location. (left of right).
       Add Function to drive to target location. Start motor.
       */
      Serial.println();
      Serial.println("-------------");
      Serial.println();
    }

  }
}

// .... Below this are functions ....

void getlocation(TinyGPS &gps) {
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
}


bool check_button() {
  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    digitalWrite(ledPin, HIGH);            // turn LED on
    return true; 
  } 
  else {
    digitalWrite(ledPin, LOW);             // turn LED off
    save_flat=0;                           // 0 out saved location lat
    save_flon=0;                           // 0 out saved location lon
    return false; 
  }

}


void save_location() {
  if ( save_flat == 0 && save_flon == 0 ) {
    save_flat=flat;
    save_flon=flon; 
    Serial.print("saved flat: "); printFloat(save_flat,5);
    Serial.println();
    Serial.print("saved flon: "); printFloat(save_flon,5);
    Serial.println();
  }
}


bool feedgps()
{
  while (nss.available())
  {
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}


float distance() 
{    
  float flat1 = flat;      // flat/flon contain current location.
  float flon1 = flon;      
  float flat2 = save_flat; //save_flat/flon contain saved location.
  float flon2 = save_flon;
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;
  
  Serial.print("current flat: "); printFloat(flat,5);
  Serial.println();
  Serial.print("current flon: "); printFloat(flon,5);
  Serial.println();
  
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

  dist_calc*=6371000.0; //Converting to meters
  //  Serial.println("distance in meters: ");
  //  Serial.println(dist_calc);
  dist_calc*=3.281;     //Converting to Feet
  //  Serial.println("distance in feet:");
  //  Serial.println(dist_calc);
  //  dist_calc/=5280;      //Converting to Miles
  //  Serial.println("distance in feet:");
  //  Serial.println(dist_calc);
  return(dist_calc);
}


float bearing() 
{  
  //-----heading formula below. Calculates heading to the waypoint from the current locaiton-----/
  float flat1=flat;            
  float flon1=flon;
  float heading=0;
  float flat2=save_flat;      // setting x2lat and x2lon equal to our first waypoint
  float flon2=save_flon; 
  flat1=radians(flat1);
  flon1=radians(flon1);  //also must be done in radians
  flat2=radians(flat2);
  flon2=radians(flon2);

  heading = atan2(sin(flon2-flon1)*cos(flat2),cos(flat1)*sin(flat2)-sin(flat1)*cos(flat2)*cos(flon2-flon1)),2*3.1415926535;
  heading = heading*180/3.1415926535;  // convert from radians to degrees
  int head =heading; 
  if(head<0){
    heading+=360;   //if the heading is negative then add 360 to make it positive
  }

  //  Serial.println("heading:");
  //  Serial.println(heading);
  return(heading);
}  


//void turn() 
// {
//  int turn=0;
//  Wire.beginTransmission(slaveAddress);        //the wire stuff is for the compass module
//  Wire.send("A");              
//  Wire.endTransmission();
//  delay(10);                  
//  Wire.requestFrom(slaveAddress, 2);       
//  i = 0;
//  while(Wire.available() && i < 2)
//  { 
//    headingData[i] = Wire.receive();
//    i++;
//  }
//  headingValue = headingData[0]*256 + headingData[1];
//  int pracheading = headingValue / 10;      // this is the heading of the compass
//  if(pracheading>0){
//    headinggps=pracheading;
//  }
//Serial.println("current heaDING:");
//Serial.println(headinggps);
//x4=headinggps-heading;   //getting the difference of our current heading to our needed heading

//Serial.println(absolute);

//  int x5;

//-----below tells us which way we need to turn
//if(x4>=-180){
//if(x4<=0){
//turn=8;    //set turn =8 which means "right"         
//}
// }
//if(x4<-180){
//turn=5;      //set turn = 5 which means "left"
//}
//if(x4>=0){
// if(x4<180){
//   turn=5;   //set turn = 5 which means "left"
// }
//}
//  if(x4>=180){     //set turn =8 which means "right"
//    turn=8;
//  }
//-----real important turning stuff. DO NOT TOUCH!!!
//  float hd = headinggps;
//  if(hd==heading){
//    turn=3;   //then set turn = 3 meaning go "straight"
//  }
//  if(turn==3){
//    servo.write(90);  //drive straight
//    esc.write(speedesc);
//    delay(60);
//  }
//--turn right
//  if(turn==8){
//    rightturn();
//  }
//--turn left
//  if(turn==5){
//    leftturn();
//  }

//-------------------------------------------------------------------------
//}



//void done(){
//  esc.write(70);  
//  done();
//}
//----right turning
//void rightturn(){
//  if(headinggps+2>heading){
//    if(headinggps-2<heading){
//    servo.write(90);
//      delay(60);
//      return;
//    }
//  }
//  x4=headinggps-heading;  
//  if(x4<-180){
//    return;
//  }
//  if(x4>=0){
//    if(x4<180){
//      return;
//    }
//  }
//  servo.write(80);
//  esc.write(speedesc);
//  Wire.beginTransmission(slaveAddress);        //the wire stuff is for the compass module
//  Wire.send("A");              
//  Wire.endTransmission();
//  delay(10);                  
//  Wire.requestFrom(slaveAddress, 2);       
//  i = 0;
//  while(Wire.available() && i < 2)
//  { 
//    headingData[i] = Wire.receive();
//    i++;
//  }
//  headingValue = headingData[0]*256 + headingData[1];  
//  headinggps = headingValue / 10;      // this is the heading of the compass
//  rightturn();
//}

//--left turning
//void leftturn(){
//  if(headinggps+4>heading){
//    if(headinggps-4<heading){
//  servo.write(90);
//      delay(60);
//      return;
//    }
//  }
//  x4=headinggps-heading;  
//  if(x4>=-180){
//    if(x4<=0){
//      return;         
//    }
//  }

//  if(x4>=180){     
//    return;
//  }
//  servo.write(110);  // turn wheels left
//  esc.write(speedesc);  //our speed

//  Wire.beginTransmission(slaveAddress);        //the wire stuff is for the compass module
//  Wire.send("A");              
//  Wire.endTransmission();
//  delay(10);                  
//  Wire.requestFrom(slaveAddress, 2);       
//  i = 0;
//  while(Wire.available() && i < 2)
//  { 
//    headingData[i] = Wire.receive();
//    i++;
//  }
//  headingValue = headingData[0]*256 + headingData[1];  
//  headinggps = headingValue / 10;      // this is the heading of the compass
//  leftturn();
//}


//void buttonread(){
//  int butval= analogRead(buttons);
//  if(waypoints>0){
//    if(butval<100){
//      buttonreaddone=5;
//      return;
//    }
//  }
//  if(waypoints==0){
//    if(butval<100){
//      radius+=1;
//      return;
//    }
//  }
//  if(butval>600){
//    if(waypoints==0){
//      flat2=flat;
//      flon2=flon;
//      waypoints+=1;
//      return;
//    }
//  }
//}

// Functions used for Debug
void gpsdump(TinyGPS &gps)
{
  long lat, lon;
  unsigned long age, date, time, chars;
  unsigned short sentences, failed;
  feedgps(); // Feed the gps during this long routine, else may get checksum errors
  gps.f_get_position(&flat, &flon, &age);
  Serial.print("Lat/Long(float): "); 
  printFloat(flat, 5); 
  Serial.print(", "); 
  printFloat(flon, 5);
  Serial.print(" Fix age: "); 
  Serial.print(age); 
  Serial.println("ms.");
  feedgps();
  Serial.print(" Course(float): ");
  printFloat(gps.f_course()); 
  feedgps();
  gps.stats(&chars, &sentences, &failed);
  Serial.print("Stats: characters: "); 
  Serial.print(chars); 
  Serial.print(" sentences: "); 
  Serial.print(sentences); 
  Serial.print(" failed checksum: "); 
  Serial.println(failed);
}

void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0)
  {
    Serial.print('-');
    number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint; 
  } 
}





