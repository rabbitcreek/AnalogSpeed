                                                                                                                                                                                                                                                         
#define STEPS 600 //This is the full swing of chosen stepper
#include <Stepper.h>
#include <TinyGPS++.h>
#define VBATPIN A7
#define badBat 10
#define goodBat 13
#define upLimit 540
TinyGPSPlus gps;
Stepper stepper(STEPS, 5, 6, 11, 12);  //These are the pins I used on the feather board

float totalDist=0;
boolean validGPS = false;
unsigned long lastUpdateTime = 0;
boolean firstLight=true;
int newAlt=0;
int newSpeed=0;








double milesDist=0;
int printDist=0;
int oldOutput=0;
int mode=0;
double distanceToDestination=0;
double runLat; //running lat position
double runLng;  //running lng position
float homeLat; //Home location  
float homeLng;  //Home location
#define ConsoleBaud 115200
#define SPREADS 40 //This is adjusted for speed in MPH on meter
#define SPREADD 40 //This is adjusted for distance in Miles on meter
#define SPREADA 4000 //this is adjusted to give total height in feet on meter face
/*In this section you can put in a location for "home" Otherwise it is taken from the 
 * initial location when you turn on the machine..
 */
 //homeLat=(put in latitude of home);
 //homeLng=(put in longitude of home);
  

void setup()
{
  Serial1.begin(9600); // or what ever the GPS's baudrate is
  //This section resets the location of the Stepper to zero position
  stepper.setSpeed(60);
  stepper.step(-600);
  stepper.step(upLimit);
  delay(1000);
  stepper.step(-600);
  //These are the pins to control the led's at the top of the meter
  pinMode(badBat, OUTPUT);
  pinMode(goodBat, OUTPUT);
  digitalWrite(badBat, LOW);
  digitalWrite(goodBat, LOW);

  
 }

void loop()
{
  if (Serial1.available())
  {
    validGPS = gps.encode(Serial1.read());
  }
  if (validGPS)
   {
  
  // The TinyGPS++ object 

  // Every 1/2 second, do an update.
   if (millis() - lastUpdateTime >= 500)
  {
    lastUpdateTime = millis();
 /*   This section is for controlling the LED's to signal battery level if needed   
float measuredvbat = analogRead(VBATPIN);
measuredvbat *= 2;    // we divided by 2, so multiply back
measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
measuredvbat /= 1024; // convert to voltage
Serial.print("voltage");
Serial.println(measuredvbat);
if(measuredvbat<3.5)
{
  digitalWrite(badBat,HIGH);
   
}
 else 
 {
  
  digitalWrite(badBat, LOW);
 }
 */

 /*This section sets the initial lat and lng to the start position 
    you can also set these points in the program if you always have 
    a home point you leave from
    */
     if(firstLight)
   {
    delay(10000);
     homeLat=gps.location.lat();
     homeLng=gps.location.lng();
     runLat=homeLat;
     runLng=homeLng;
     Serial.print("homeLat");
     Serial.println(homeLat);
     delay(10000);
     if(runLat!=0&&runLng!=0)
     {
     firstLight=false;
     }
     
    }
    //Location and distance section
 if(gps.location.isUpdated())
 {     
      distanceToDestination = TinyGPSPlus::distanceBetween(
      gps.location.lat(), gps.location.lng(),runLat, runLng);
      Serial.print("gps lat");
      Serial.println(gps.location.lat());
      Serial.print("gps lng");
      Serial.println(gps.location.lng());
      Serial.print("runLat");
      Serial.print(runLat);
      Serial.print("runLng");
      Serial.print(runLng);
      delay(50);
      Serial.print("distanceToDestination");
      Serial.println(distanceToDestination);
      //Bites off 200 meter pieces of distance and sets new home coordinates to next
      //200 meter section--
      if(distanceToDestination>200)
      {
       totalDist=totalDist+distanceToDestination;
       runLat=gps.location.lat();
       runLng=gps.location.lng();
       delay(50);
       Serial.print("total distance");
       Serial.println(totalDist);
       //meter to mile conversion
       milesDist=totalDist*(0.00062);
      printDist=map(milesDist,0,SPREADD,0,upLimit);
      printDist=constrain(printDist,0,upLimit);
      delay(50);
      Serial.print("printDist");
      Serial.println(printDist);
      Serial.print("milesDist");
      Serial.println(milesDist);
      }
        
      
 }
 //speed section
     if(gps.speed.isUpdated())
    {
     
      newSpeed=gps.speed.mph();
      newSpeed=map(newSpeed,0,SPREADS,0,upLimit);
      newSpeed=constrain(newSpeed,0,upLimit);
      Serial.print("speed in Mph");
      Serial.println(newSpeed);
   
    }
    //altitude section--not used currently
    if(gps.altitude.isUpdated())
    {
      newAlt=gps.altitude.feet();
      newAlt=map(newAlt,0,SPREADA,0,upLimit);
      newAlt=constrain(newAlt,0,upLimit);
      Serial.print("altitude");
      Serial.println(newAlt);
         
     }
/*Tells the meter face which output it is getting. In this case
 * only speed and distance are outputed with the corresponding LED 
 * light to activate--green for speed and red for stop when distance
 * is displayed
 */

   if(newSpeed<1)mode=1;
    if(mode==0)
    {
    stepper.step(newSpeed-oldOutput);
    oldOutput=newSpeed;
    digitalWrite(goodBat,HIGH);
    digitalWrite(badBat,LOW);
    
    }
    if(mode==1)
    {
      stepper.step(printDist-oldOutput);
      oldOutput=printDist;
      digitalWrite(badBat,HIGH);
      digitalWrite(goodBat,LOW);
      
    }
    //mode never becomes 2 --altitude not being used
    if (mode==2)
    {
      stepper.step(newAlt-oldOutput);
      oldOutput=newAlt;
    }
    mode=0;
  }
   }
   

 }

    

