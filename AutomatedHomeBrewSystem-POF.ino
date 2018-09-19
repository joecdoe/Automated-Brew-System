/* 
 *  The project aimed to provide a proof of concept for automating the home beer brewing process through a system of valves and 
 *  pumps controlled using Arduino. The system is capable of monitoring temperature of the fluid and uses a series of relays to 
 *  automate the brew process. Transfering between tanks is determined based on a temperature change of deltaT. Cooling of the 
 *  liquid is carried out by using an external water to pump cool liquid into the cooling coil. An ignitor electrode used for 
 *  BBQ grills was used to cause the initial spark for igniting the flow of propane at the burner. The temperature change of the 
 *  liquid and the current temperature of the liquid are programmed to be displayed on a 20x4 LCD display.
 *  
 *  Future upgrades would require to include a volume sensor for each tank as well as flow meters to determine timing the pumps
 *  to be turned on and off based on fluid present. 
 *  
 *  Presented to University of the Pacific School of Engineering May 2018
 *  
 */

#include <LiquidCrystal.h>
#include <max6675.h>
#include <SPI.h>
#include <SPI.h>

// Thermocouple 1
/*#define MAX6675_SCK1 53 //thermocouple 1
#define MAX6675_CS1  51 //thermocouple 1
#define MAX6675_SO1  49 //thermocouple 1
*/
int thermo_sck_pin = 53;
int thermo_cs_pin = 51;
int thermo_so_pin = 49;
int thermo_vcc_pin=47; 
int thermo_gnd_pin=45;

MAX6675 thermocouple(thermo_sck_pin, thermo_cs_pin, thermo_so_pin);


// Thermocouple 2
/*
#define MAX6675_SCK2 47 //thermocouple 2
#define MAX6675_CS2  45 //thermocouple 2
#define MAX6675_SO2  43 //thermocouple 2
*/

int thermo2_sck_pin = 41;
int thermo2_cs_pin = 39;
int thermo2_so_pin = 37;
int thermo2_vcc_pin=35; 
int thermo2_gnd_pin=33;

MAX6675 thermocouple2(thermo2_sck_pin, thermo2_cs_pin, thermo2_so_pin);

const int lvalve1 = 23;  // liquid valve at tank 1
const int lvalve2 = 25;  //final valve to empty into storage tank

const int pvalve1 = 27; //Propane2 valve Relay for tank 2
const int pvalve2 = 29; //Propane2 valve Relay for tank 2


const int ignitor1 = 11; //spark ignitor electrode
const int flamesensor1 = 13; // flame sensor for steep tank
const int flamesensor2 = 12; // flame sensor for tank 2
const int pump1 = 22;
const int pump2 = 24;

int digitalVal1;
int digitalVal2;
float delta1;
float delta2;
float delta3;
float TempVal1;
float TempVal2;
float TempVal3;
float temp1;
float temp2;
float temp3;
int counter;
int pumptime;
int tlimit;




void setup() {
  Serial.begin(9600);
  digitalVal1 = digitalRead(flamesensor1);
  digitalVal2 = digitalRead(flamesensor2);

  pinMode(lvalve1, OUTPUT); // connected to S terminal of liquid solenoid 1 Relay
  pinMode(lvalve2, OUTPUT); // connected to S terminal of liquid solenoid 2 Relay
  
  pinMode(pump1, OUTPUT); // connected to S terminal of pump 1 Relay
  pinMode(pump2, OUTPUT); // connected to S terminal of pump 2 Relay

  pinMode(flamesensor1, INPUT); // flame sensor for steep tank
  pinMode(flamesensor2, INPUT); // flame sensor for boil tank
  pinMode(ignitor1, OUTPUT);    // spark ignitors for both burners

  pinMode(pvalve1, OUTPUT);   //propane valve 1
  pinMode(pvalve2, OUTPUT);   // propane valve 2

// Thermocouple 1 pins
  pinMode(thermo_vcc_pin, OUTPUT);
  pinMode(thermo_gnd_pin, OUTPUT);
  digitalWrite(thermo_vcc_pin, HIGH);
  digitalWrite(thermo_gnd_pin, LOW);

// Thermocouple 2 pins
  pinMode(thermo2_vcc_pin, OUTPUT);
  pinMode(thermo2_gnd_pin, OUTPUT);
  digitalWrite(thermo2_vcc_pin, HIGH);
  digitalWrite(thermo2_gnd_pin, LOW);

  delay(5000);

// BEGIN PROGRAM
   Serial.print("Begin Program\n");

// Take First tank initial temperature reading   
 for(int i=0;i<3;i++)
  {
    temp1 = thermocouple.readCelsius();
    delay(1000);
    Serial.print("temp1 =");
    Serial.println(temp1); 
    delay(1000);
  }
   Serial.print("\nfinal start temp =");
   Serial.print(temp1);

 
  delay(2000);
  heating1();
  delay(2000);
  transfer1(); //run one time
  delay(2000);

// Take Second tank initial temperature reading
 for(int i=0;i<3;i++)
  {
    temp2 = thermocouple2.readCelsius();
    delay(1000);
    Serial.print("temp2 =");
    Serial.println(temp2); 
    delay(1000);
  }
   Serial.print("\nfinal start temp =");
   Serial.print(temp2);  

  delay(2000);
  
  heating2();

  Serial.print("\nHeating Done");
  digitalWrite(pvalve2, LOW);
  delay(2000);
  Serial.println("\nBegin Cooling Stage");
  delay(100);


// Take Initial temperature of tank 2 for cooling
 for(int i=0;i<3;i++)
  {
    temp3 = thermocouple2.readCelsius();
    delay(1000);
    Serial.print("temp3 =");
    Serial.println(temp3); 
    delay(1000);
  }

  cooling();
  Serial.println("\nCooling Stage completed");
  digitalWrite(pump2, LOW);         // Ensure pum 2 is closed
  delay(100);

  Serial.println("\nBegin final Transfer Stage");
  delay(1000);
  
  transfer2();
 

}


////////////////////////// Void loop
void loop() {
exit(0);
}


//////////////////////////// Heating 1
void heating1() {

 //   Serial.print("Temperature: ");
 //   Serial.print(thermocouple.readCelsius());
 //   Serial.println('C');
     Serial.println("\nBEGIN HEATING 1 STAGE");
     delay(500);
     
     digitalWrite(pvalve1, HIGH); 
     Serial.println("\nPropane valve 1 open");
     delay(500); 

     digitalWrite(ignitor1, HIGH);  //spark ignitor for 1 sec
     delay(1000);
     digitalWrite(ignitor1,LOW);
     delay(2000);                   // Pause to dissapate energy
                
  while(delta1 <= 5 && counter < 120)                      // If +2 degree change heating 1 complete
   {
    TempVal1 = thermocouple.readCelsius();        // Take temperature reading
    Serial.print("\ntempVal1 =");
    Serial.println(TempVal1);
    delta1   = abs(TempVal1 - temp1a);                  // Compare to initial temperature reading
    Serial.print("delta1 =");
    Serial.println(delta1);
    counter = counter+1;
    Serial.print("counter =");
    Serial.println(counter);

    delay(1000);      
   }
  }    
////////////////////////////// Liquid Transfer 1
void transfer1(){
     Serial.println("\nTransfering into tank 2");
     digitalWrite(pvalve1, LOW); //turn off burner
     delay(3000);
     digitalWrite(lvalve1, HIGH); //open liquid transfer valve to prime pump
     delay(5000); //5 sec pause to let valve open and prime pump
     digitalWrite(pump1, HIGH); //turn on pump

     counter = 0;
     tlimit= 38;
       for(counter;counter<=tlimit;counter++)     // Count down for pump = 30 seconds on
       {
        pumptime = tlimit - counter;
        Serial.print("transfer time remaining:");
        Serial.println(pumptime);
        delay(1000);
       }
//    delay(35000); //keep pump 1 on for 1 mins (time how long the pump needs to stay on for)
     digitalWrite(pump1, LOW); // done transfering, turn off pump 1
     delay(1000);
     digitalWrite(lvalve1, LOW); //close valve 1
  delay(1000);
}



/////////////////////////// Heating 2
void heating2() {
 // Serial.print(thermocouple2.readCelsius());
 // Serial.println('C');
  counter = 0;
  digitalWrite(pvalve2, HIGH); 
  Serial.println("\nPropane valve 2 open");
  delay(500); 
  
  digitalWrite(ignitor1, HIGH);  //spark ignitor for 3 sec
  delay(1000);
  digitalWrite(ignitor1,LOW);
  delay(2000);                   // Wait 2 seconds to dissipate any energy for thermocouple

  
  Serial.println("Begin Boiling Stage\n");
  while (delta2 <= 5  && counter < 120)                      // Continue to cooling if +2 degree change
  {
    TempVal2 = thermocouple2.readCelsius();       // Take temperature reading Value at tank 2
    Serial.print("\nTempval2 =");
    Serial.println(TempVal2);
    delta2   = abs(TempVal2 - temp2);                  // Compare to initial temperature reading taken
    Serial.print("Delta2 =");
    Serial.println(delta2); 
    counter = counter + 1; 
    
    Serial.print("counter =");
    Serial.println(counter);
    delay(1000);
  }
  delay(1500);
}

///////////////////////////// Tank 2 Cooling
void cooling() {
  Serial.println("\nCOOLING");
  counter = 0;
  digitalWrite(pvalve2, LOW); //make sure propane valve 2 is closed
  delay(1000);
  digitalWrite(pump2, HIGH); //Pump2 on, begin pumping cold water through coil    
  
  while(delta3 <= 5 && counter < 120)              // when temperature of tank drops 3 degrees continue
  {
    TempVal3 = thermocouple2.readCelsius();     // Take temperature reading of tank 2
    Serial.print("\nTempVal3 =");
    Serial.println(TempVal3);
    
    delta3   =  abs(temp3 - TempVal3);               // Compare to initial temperature reading
    Serial.print("delta3 =");
    Serial.println(delta3);
    
    counter = counter + 1; 
    Serial.print("counter=");
    Serial.println(counter);
    delay(1000); //cooling coil on (checks every 5 sec)
  }
  digitalWrite(pump2, LOW);       // Turn off pump 2 once liquid is cooled
}

 ////////////////////////////// Liquid Transfer 2
 void transfer2() {
    Serial.println("\nCooled Transfering to Storage tank");
    digitalWrite(lvalve2, HIGH); //Opem valve to transfer to final storage tank
    
    counter = 0;
    tlimit = 150;
    pumptime = 0;

    for(counter;counter<=tlimit;counter++)
    {
      pumptime = tlimit-counter;
      Serial.print("time left:");
      Serial.println(pumptime);
      delay(1000);
    }
    
 //   delay(180000); //tranfer valve open for (change to 3 min) to gravity feed into final tank
    digitalWrite(lvalve2, LOW);
    Serial.println("Demonstration Complete");
    delay(2000);
    exit(0);
 }




/////////////// Code for thermocouple 1
/*double readThermocouple1() {

  uint16_t v;
  pinMode(MAX6675_CS1, OUTPUT);
  pinMode(MAX6675_SO1, INPUT);
  pinMode(MAX6675_SCK1, OUTPUT);
  
  digitalWrite(MAX6675_CS1, LOW);
  delay(1);

  // Read in 16 bits,
  //  15    = 0 always
  //  14..2 = 0.25 degree counts MSB First
  //  2     = 1 if thermocouple is open circuit  
  //  1..0  = uninteresting status
  
  v = shiftIn(MAX6675_SO1, MAX6675_SCK1, MSBFIRST);
  v <<= 8;
  v |= shiftIn(MAX6675_SO1, MAX6675_SCK1, MSBFIRST);
  
  digitalWrite(MAX6675_CS1, HIGH);
  if (v & 0x4) 
  {    
    // Bit 2 indicates if the thermocouple is disconnected
    return NAN;     
  }

  // The lower three bits (0,1,2) are discarded status bits
  v >>= 3;

  // The remaining bits are the number of 0.25 degree (C) counts
  return v*0.25;
}


/////////////// Code for thermocouple 2
double readThermocouple() {

  uint16_t v;
  pinMode(MAX6675_CS2, OUTPUT);
  pinMode(MAX6675_SO2, INPUT);
  pinMode(MAX6675_SCK2, OUTPUT);
  
  digitalWrite(MAX6675_CS2, LOW);
  delay(1);

  // Read in 16 bits,
  //  15    = 0 always
  //  14..2 = 0.25 degree counts MSB First
  //  2     = 1 if thermocouple is open circuit  
  //  1..0  = uninteresting status
  
  v = shiftIn(MAX6675_SO2, MAX6675_SCK2, MSBFIRST);
  v <<= 8;
  v |= shiftIn(MAX6675_SO2, MAX6675_SCK2, MSBFIRST);
  
  digitalWrite(MAX6675_CS2, HIGH);
  if (v & 0x4) 
  {    
    // Bit 2 indicates if the thermocouple is disconnected
    return NAN;     
  }

  // The lower three bits (0,1,2) are discarded status bits
  v >>= 3;

  // The remaining bits are the number of 0.25 degree (C) counts
  return v*0.25;
}

*/

