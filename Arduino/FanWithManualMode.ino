//Control fan speed based on thermosistor reading and also 
//display fan speed RMP in Arduino Serial Monitor

//Baud should be 125K

//////////////////////////////////////////////////////////////////////////////////////
#include <math.h>


#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>


#define FlowThermistorPIN 0                 // Analog Pin 0
#define FlatThermistorPIN 5
#define FANPIN 9  // Takes dPin#9 supports PWM
#define HALLSENSOR 2


# define BTNBLUE 7 //+
# define BTNRED 8 //-
# define FANSWITCH 4 //Fan switch
int manFanSpeed = 130;



//Varibles used for calculations
int NbTopsFan; 
int Calc;
//The pin location of the sensor
//int HALLSENSOR = 2;
//int fanPWM = 9;

typedef struct{                  //Defines the structure for multiple fans and their dividers
  char fantype;
  unsigned int fandiv;
}fanspec;

//Definitions of the fans
fanspec fanspace[3]={{0,1},{1,2},{2,8}};

char fan = 1;   //This is the varible used to select the fan and it's divider, set 1 for unipole hall effect sensor 
//and 2 for bipole hall effect sensor 


void rpm ()      //This is the function that the interupt calls 
{ 
  NbTopsFan++; 
} 



float vcc = 5.00;                       // only used for display purposes, if used
// set to the measured Vcc.
float pad = 10000;                       // balance/pad resistor value, set this to
// the measured resistance of your pad resistor
float thermr = 10460;                   // thermistor nominal resistance



int fanSpeedHIGH = 250;  // 20 to 255 as fan speed
double fanSpeedHIGHThreshold = 28.0;

int fanSpeedLOW = 20;


float Thermistor(int RawADC) {
  long Resistance;  
  float Temp;  // Dual-Purpose variable to save space.

  Resistance=pad*((1024.0 / RawADC) - 1); 
  Temp = log(Resistance); // Saving the Log(resistance) so not to calculate  it 4 times later
  Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
  Temp = Temp - 273.15;  // Convert Kelvin to Celsius                      

  // BEGIN- Remove these lines for the function not to display anything
  //Serial.print("ADC: "); 
  //Serial.print(RawADC); 
  //Serial.print("/1024");                           // Print out RAW ADC Number
  //Serial.print(", vcc: ");
  //Serial.print(vcc,2);
  //Serial.print(", pad: ");
  //Serial.print(pad/1000,3);
  //Serial.print(" Kohms, Volts: "); 
  //Serial.print(((RawADC*vcc)/1024.0),3);   
  //Serial.print(", Resistance: "); 
  //Serial.print(Resistance);
  //Serial.print(" ohms, ");
  // END- Remove these lines for the function not to display anything

  // Uncomment this line for the function to return Fahrenheit instead.
  //temp = (Temp * 9.0)/ 5.0 + 32.0;                  // Convert to Fahrenheit
  return Temp;                                      // Return the Temperature
}

void setup() {
  Serial.begin(9600);
  pinMode(BTNBLUE, INPUT);
  pinMode(BTNRED, INPUT);
  pinMode(FANSWITCH, INPUT);


  pinMode(FANPIN, OUTPUT);
  pinMode(HALLSENSOR, INPUT); 
  //pinMode(fanPWM,OUTPUT);
  attachInterrupt(0, rpm, RISING); 


  Serial.println("CAN Write - Testing transmission of CAN Bus messages");
  delay(1000);

  if(Canbus.init(CANSPEED_125))  //Initialise MCP2515 CAN controller at the specified speed
  Serial.println("CAN Init ok");
  else
  Serial.println("Can't init CAN");

  delay(1000);

}

void loop() {
  float temp;
  temp=Thermistor(analogRead(FlowThermistorPIN));       // read ADC and  convert it to Celsius
  Serial.print("FLOW Thermistor Celsius: "); 
  Serial.print(temp,1);                             // display Celsius
  //temp = (temp * 9.0)/ 5.0 + 32.0;                  // converts to  Fahrenheit
  //Serial.print(", Fahrenheit: "); 
  //Serial.print(temp,1);                             // display  Fahrenheit
  Serial.println("");  



  float reading;

  reading = analogRead(FlowThermistorPIN);

  Serial.print("FLOW Thermistor Analog reading "); 
  Serial.println(reading,1);

  // convert the value to resistance
  reading = (1023 / reading)  - 1;
  reading = pad / reading;
  Serial.print("FLOW Thermistor resistance "); 
  Serial.println(reading);


  bool switchFan;
  if (digitalRead(FANSWITCH) == HIGH) {
    switchFan = true;
  } else {
    switchFan = false;
  }


  if (switchFan == false) {

    if(temp>fanSpeedHIGHThreshold){

      analogWrite(FANPIN, fanSpeedHIGH);
      Serial.println("Fan speed is HIGH");
      Serial.println("      ");

    } else {
      analogWrite(FANPIN,fanSpeedLOW);
      Serial.println("Fan speed is LOW");
      Serial.println("      ");

      
  //=================

  tCAN message;

  //send state
  message.id = 0x0F0; //formatted in HEX
  message.header.rtr = 0;
  message.header.length = 2; //formatted in DEC
  message.data[0] = 0x4C; //L
  message.data[1] = 0x4F; //O
  //  message.data[2] = 0x30;
  //  message.data[3] = 0xFF; //formatted in HEX
  //  message.data[4] = 0x00;
  //  message.data[5] = 0x40;
  //  message.data[6] = 0x00;
  //  message.data[7] = 0x00;

  mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
  mcp2515_send_message(&message);


  //delay(1000);

  //send out fluid temp
  //tCAN message;

  message.id = 0x0F1; //formatted in HEX
  message.header.rtr = 0;
  message.header.length = 4; //formatted in DEC

  int temp1 = (int)temp;
  message.data[0] = (temp1>>24) & 0xFF;
  message.data[1] = (temp1>>16) & 0xFF;
  message.data[2] = (temp1>>8) & 0xFF;
  message.data[3] = (temp1) & 0xFF;
  mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
  mcp2515_send_message(&message);

  delay(1000);


  //===========================


    }
  } else {

    if( ( (digitalRead(BTNRED) == LOW)  && (digitalRead(BTNBLUE) == LOW) )){
      analogWrite(FANPIN, manFanSpeed);
      Serial.print("MANUAL Mode: No changes. Fan speed is: ");
      Serial.println(manFanSpeed);
    }
    
    else if(( (digitalRead(BTNRED) == LOW)  && (digitalRead(BTNBLUE) == HIGH) ) ){
      manFanSpeed = manFanSpeed + 10;
      if(manFanSpeed <= 255) {manFanSpeed = manFanSpeed;}
      else {manFanSpeed = 255;}
      analogWrite(FANPIN, manFanSpeed);
      Serial.print("MANUAL Mode: Fan speed is ++ : ");
      Serial.println(manFanSpeed);

    }
    else if(( (digitalRead(BTNRED) == HIGH)  && (digitalRead(BTNBLUE) == LOW) )){
      manFanSpeed = manFanSpeed - 10;
      if(manFanSpeed >= 10) {manFanSpeed = manFanSpeed;}
      else {manFanSpeed = 10;}
      analogWrite(FANPIN, manFanSpeed);
      Serial.print("MANUAL Mode: Fan speed is -- : " );
      Serial.println(manFanSpeed);

    }

    else {
      analogWrite(FANPIN, manFanSpeed);
      Serial.println("MANUAL Mode: No changes (else).");
      Serial.println(manFanSpeed);
    }
  } 










  //=================================
  float tempFlat;
  tempFlat=Thermistor(analogRead(FlatThermistorPIN));       // read ADC and  convert it to Celsius
  Serial.print("FLAT Thermistor Celsius: "); 
  Serial.print(tempFlat,1);                             // display Celsius
  //temp = (temp * 9.0)/ 5.0 + 32.0;                  // converts to  Fahrenheit
  //Serial.print(", Fahrenheit: "); 
  //Serial.print(temp,1);                             // display  Fahrenheit
  Serial.println("");  



  float readingFlat;

  readingFlat = analogRead(FlatThermistorPIN);

  Serial.print("FLAT Thermistor Analog reading "); 
  Serial.println(readingFlat);

  // convert the value to resistance
  readingFlat = (1023 / readingFlat)  - 1;
  readingFlat = pad / readingFlat;
  Serial.print("FLAT Thermistor resistance "); 
  Serial.println(readingFlat);





  NbTopsFan = 0;  //Set NbTops to 0 ready for calculations
  sei();   //Enables interrupts
  delay (1000);  //Wait 1 second
  cli();   //Disable interrupts
  Calc = ((NbTopsFan * 60)/fanspace[fan].fandiv); //Times NbTopsFan (which is apprioxiamately the fequency the fan is spinning at) by 60 seconds before dividing by the fan's divider
  Serial.print (Calc, DEC); //Prints the number calculated above
  Serial.print (" Fan RPM\r\n"); //Prints " rpm" and a new line



  Serial.println("-------------------------");


  delay(1000);                                      // Delay a bit... 
}



