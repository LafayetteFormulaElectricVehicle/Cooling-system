/*
* Cooling System Central Controller
* Created by Xingyuan Guo for ECE 492, Spring 2017, Lafayette College
* guoxyxy at gmail
*
* Due to timing limit, this code is not created as Object-Oriented,
* which can be further changed to provide easy understanding.
*
* References and explanations of some of the code can be found at the end.
*/






//===================================================
//Starts Internal/External Library usage=========================
#include <math.h>
#include <Canbus.h>   //Canbus and MCP2515 libraries are external
#include <defaults.h> //, which should be imported first to Arduino
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>
#include <SoftwareSerial.h>
//Ends Internal/ExternalLibrary usage=========================
//===================================================






//===================================================
//Starts Flow Meter Constants and Pins=========================
#define FLOW_PIN 3
#define INTERVAL 1000

boolean latch1_, latch2_ = false;
unsigned int window_count_ = 0;
unsigned long total_count_ = 0;

unsigned long current_millis_ = 0;
unsigned int current_frequency_ = 0;
//Ends Flow Meter Constants and Pins=========================
//===================================================






//===================================================
//Starts Thermistor Constants and Pins=========================
#define FlowThermistorPIN 0  // Analog Pin 0
#define FlatThermistorPIN 5  //Analog Pin 5
float pad = 10000;           // balance/pad resistor value, set this to
// the measured resistance of your pad resistor
float thermr = 10460;        // thermistor nominal resistance

//Global TEMP parameter
float tempFlow, resistanceFlow;

float tempFlat, resistanceFlat;

//Ends Thermistor Constants and Pins=========================
//===================================================






//===================================================
//Starts Fan Control Constants and Pins=========================
#define FANPIN 9  // Takes dPin#9 supports PWM
int manFanSpeed = 130;
int fanSpeedHIGH = 250;  // 20 to 255 as fan speed
double fanSpeedHIGHThreshold = 28.0;
int fanSpeedLOW = 50;

//Global parameter
bool switchFan;
bool modeSelection;
//Ends Fan Control Constants and Pins=========================
//===================================================






//===================================================
//Starts Fan RPM Reading Constants and Pins=========================
#define fanPulse 12
unsigned long pulseDuration;
//Ends Fan RPM Reading Constants and Pins=========================
//===================================================






//===================================================
//Starts Buttons/Switches Constants and Pins=========================
# define BTNBLUE 7 //+ during manual mode
# define BTNRED 8 //- during manual mode
# define FANSWITCH 4 //Fan switch
//Ends Buttons/Switches Constants and Pins=========================
//===================================================






//===================================================
//Starts LCD Display Constants and Pins=========================
SoftwareSerial mySerial(3,6); //pin 6 = TX, pin 3 = RX (unused)
//Ends LCD Display Constants and Pins=========================
//===================================================






//===================================================
//Starts LCD Constants and Pins=========================
const int LCDdelay=5;  // conservative, 2 actually works
//Ends LCD Functions=========================
//===================================================






//===================================================
//Starts Arduino Setup Function=========================
void setup() {
	Serial.begin(9600);
	pinMode(BTNBLUE, INPUT);
	pinMode(BTNRED, INPUT);
	pinMode(FANSWITCH, INPUT);

	pinMode(FANPIN, OUTPUT);

	//*********************new fan rpm
	pinMode(fanPulse, INPUT);
	digitalWrite(fanPulse,HIGH);
	//*********************new fan rpm

	Serial.println("CAN Write - Testing transmission of CAN Bus messages");
	delay(1000);

	if(Canbus.init(CANSPEED_125))  //Initialise MCP2515 CAN controller at the specified speed
	Serial.println("CAN Init ok");
	else
	Serial.println("Can't init CAN");

	//================LCD
	mySerial.begin(9600); // set up serial port for 9600 baud
	//================LCD

	//delay(1000);
}
//Ends Arduino Setup Function=========================


//Starts Arduino Loop Function=========================
void loop() {

	detectFlowmeterRotation(FLOW_PIN);
	calculateFlowmeterFrequency();

	FlowThermistorReadAndPrint();
	FanModeControl();
	FlatThermistorReadAndPrint();

	ShowOnLCD();
	
	CANwrite();
	
	//clearLCD();

}
//Ends Arduino Loop Function=========================
//===================================================






//===================================================
//Starts FlowThermistorReadAndPrint() Function=========================
//In which the method measures and stores temperature of the flow thermistor
//in the global variables. Also use Serial to print useful information on scree.
void FlowThermistorReadAndPrint() {
	static unsigned long previous_millis;
	int interval = 1000;

	if (millis() - previous_millis > interval) {
		previous_millis = millis();
		
		tempFlow=Thermistor(analogRead(FlowThermistorPIN)); // read ADC and  convert it to Celsius
		Serial.print("FLOW Thermistor Celsius: "); 
		Serial.print(tempFlow,1);                  // display Celsius
		//tempFlow = (tempFlow * 9.0)/ 5.0 + 32.0;    // converts to  Fahrenheit
		//Serial.print(", Fahrenheit: "); 
		//Serial.print(tempFlow,1);     // display  Fahrenheit
		Serial.println("");  

		resistanceFlow = analogRead(FlowThermistorPIN);

		Serial.print("FLOW Thermistor Analog resistanceFlow "); 
		Serial.println(resistanceFlow,1);

		// convert the value to resistance
		resistanceFlow = (1023 / resistanceFlow)  - 1;
		resistanceFlow = pad / resistanceFlow;
		Serial.print("FLOW Thermistor resistance "); 
		Serial.println(resistanceFlow);
	}
}
//Ends FlowThermistorReadAndPrint() Function=========================
//===================================================






//===================================================
//Starts FanModeControl() Function=========================
void FanModeControl() {

	static unsigned long previous_millis;
	int interval = 1000;

	if (millis() - previous_millis > interval) {
		previous_millis = millis();
		
		if (digitalRead(FANSWITCH) == HIGH) {
			switchFan = true;
		} else {
			switchFan = false;
		}
		
		if (switchFan == false) {

			modeSelection = tempFlow > fanSpeedHIGHThreshold;
			
			if(modeSelection){

				Serial.println("Auto Mode");

				analogWrite(FANPIN, fanSpeedHIGH);
				Serial.println("Fan speed is HIGH");
				Serial.println("      ");

			} else {
				Serial.println("Auto Mode");

				analogWrite(FANPIN,fanSpeedLOW);
				Serial.println("Fan speed is LOW");
				Serial.println("      ");
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
	}
}
//Ends FanModeControl() Function=========================
//===================================================





//===================================================
//Starts FlatThermistorReadAndPrint() Function=========================
//In which the method measures and stores temperature of the flat thermistor
//in the global variables. Also use Serial to print useful information on scree.
void FlatThermistorReadAndPrint() {
	static unsigned long previous_millis;
	int interval = 1000;

	if (millis() - previous_millis > interval) {
		previous_millis = millis();
		
		tempFlat=Thermistor(analogRead(FlatThermistorPIN));       // read ADC and  convert it to Celsius
		Serial.print("FLAT Thermistor Celsius: "); 
		Serial.print(tempFlat,1);                             // display Celsius
		//temp = (temp * 9.0)/ 5.0 + 32.0;                  // converts to  Fahrenheit
		//Serial.print(", Fahrenheit: "); 
		//Serial.print(temp,1);                             // display  Fahrenheit
		Serial.println("");  



		resistanceFlat = analogRead(FlatThermistorPIN);

		Serial.print("FLAT Thermistor Analog reading "); 
		Serial.println(resistanceFlat);

		// convert the value to resistance
		resistanceFlat = (1023 / resistanceFlat)  - 1;
		resistanceFlat = pad / resistanceFlat;
		Serial.print("FLAT Thermistor resistance "); 
		Serial.println(resistanceFlat);
		

	}
}
//Ends FlatThermistorReadAndPrint() Function=========================
//===================================================






//===================================================
//Starts Flow Meter Functions=========================
void detectFlowmeterRotation(int flow_pin){
	if(digitalRead(flow_pin))
	latch1_=true;
	if(!digitalRead(flow_pin))
	latch2_=true;

	if(latch1_ && latch2_){//only true after a rotation
		latch1_ = false;
		latch2_ = false;
		window_count_++;
		total_count_++;
	}
}

void calculateFlowmeterFrequency(){

	static unsigned long previous_millis;

	if (millis() - previous_millis > INTERVAL) { //if within the rolling window INTERVAL
		previous_millis = millis(); //save last time the clock updated in previous_millis
		current_frequency_ = window_count_ * (1000/INTERVAL);
		window_count_ = 0; //reset window counter, cumulative counters unaffected.
		Serial.print("total_count_: ");
		Serial.print(total_count_);
		Serial.print(" | ");
		Serial.print("window_count_: ");
		Serial.print(current_frequency_);
		Serial.println();
		
	}
}

//Ends Flow Meter Functions=========================
//===================================================






//===================================================
//Starts LCDWorking() Function=========================
void ShowOnLCD(){

	static unsigned long previous_millis;
	int interval = 1000;

	if (millis() - previous_millis > interval) {
		previous_millis = millis();
		
		setPosition(1,0);
		mySerial.print("FlowFreq:");
		setPosition(2,0); 
		mySerial.print(current_frequency_,1);
		
		setPosition(1,10);
		mySerial.print("TempFlat:");
		setPosition(2,10); 
		mySerial.print(tempFlat,3);
		
		setPosition(3,0);
		mySerial.print("TempFlow:");
		setPosition(4,0); 
		mySerial.print(tempFlow,3);
		
		if(switchFan){
			setPosition(1,9); mySerial.print("M");
			setPosition(2,9); mySerial.print("A");
			setPosition(3,9); mySerial.print("N");
			setPosition(4,9); mySerial.print("U");
		} else {
			setPosition(1,9); mySerial.print("A");
			setPosition(2,9); mySerial.print("U");
			setPosition(3,9); mySerial.print("T");
			setPosition(4,9); mySerial.print("O");	
		}
	}
}
//Ends LCDWorking() Function=========================
//===================================================






//===================================================
//Starts CANwrite Function=========================
void CANwrite() {
	
	
	static unsigned long previous_millis;
	int interval = 1000;

	if (millis() - previous_millis > interval) {
		previous_millis = millis();
		
		
		tCAN message;

		//send state
		message.id = 0x0F0; //formatted in HEX
		message.header.rtr = 0;
		message.header.length = 2; //formatted in DEC
		
		if(!modeSelection){
			message.data[0] = 0x4C; //L
			message.data[1] = 0x4F; //O
		} else {
			message.data[0] = 0x48; //H
			message.data[1] = 0x49; //I
		}


		mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
		mcp2515_send_message(&message);


		//delay(1000);

		//send out flat temp
		message.id = 0x0F1; //formatted in HEX
		message.header.rtr = 0;
		message.header.length = 4; //formatted in DEC

		int temp1 = (int)tempFlat;
		message.data[0] = (temp1>>24) & 0xFF;
		message.data[1] = (temp1>>16) & 0xFF;
		message.data[2] = (temp1>>8) & 0xFF;
		message.data[3] = (temp1) & 0xFF;
		
		mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
		mcp2515_send_message(&message);

	}

}
//Ends CANwrite Functions=========================
//===================================================







//===================================================
//Starts Thermistor Calculation Functions=========================
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
	//tempFlow = (Temp * 9.0)/ 5.0 + 32.0;                  // Convert to Fahrenheit
	return Temp;                                      // Return the Temperature
}
//Ends Thermistor Calculation Functions=========================
//===================================================






//===================================================
//Starts LCD Functions=========================
// wbp: goto with row & column
void lcdPosition(int row, int col) {
	mySerial.write(0xFE);   //command flag
	mySerial.write((col + row*64 + 128));    //position 
	delay(LCDdelay);
}

void setPosition(int row, int col){
	int pos; 
	
	if (row==1) {
		pos=col;
	} else if (row==2) {
		pos=col+64; 
	} else if (row==3) {
		pos=col+20; 
	} else if (row==4) {
		pos=col+84;
	}
	
	pos = pos+128;
	mySerial.write(0XFE);
	mySerial.write(pos);
	delay(LCDdelay);
}

void clearLCD(){
	mySerial.write(0xFE);   //command flag
	mySerial.write(0x01);   //clear command.
	delay(LCDdelay);
}
void backlightOn() {  //turns on the backlight
	mySerial.write(0x7C);   //command flag for backlight stuff
	mySerial.write(157);    //light level.
	delay(LCDdelay);
}
void backlightOff(){  //turns off the backlight
	mySerial.write(0x7C);   //command flag for backlight stuff
	mySerial.write(128);     //light level for off.
	delay(LCDdelay);
}
void serCommand(){   //a general function to call the command flag for issuing all other commands   
	mySerial.write(0xFE);
}
//Ends LCD Functions=========================
//===================================================





















//===================================================
//Starts References =========================
//https://learn.adafruit.com/multi-tasking-the-arduino-part-1/all-together-now
//https://github.com/scogswell/ArduinoSerLCD/blob/master/examples/SerLCD_Demo_20x4/SerLCD_Demo_20x4.ino
//http://labs.teague.com/?p=722
//http://playground.arduino.cc/Learning/SparkFunSerLCD
//void clearLCD(){
//   Serial.write(0xFE);   //command flag
//   Serial.write(0x01);   //clear command.
//}
//void backlightOn(){  //turns on the backlight
//    Serial.write(0x7C);   //command flag for backlight stuff
//    Serial.write(157);    //light level.
//}
//void backlightOff(){  //turns off the backlight
//    Serial.write(0x7C);   //command flag for backlight stuff
//    Serial.write(128);     //light level for off.
//}
//void backlight50(){  //sets the backlight at 50% brightness
//    Serial.write(0x7C);   //command flag for backlight stuff
//    Serial.write(143);     //light level for off.
//}
//void serCommand(){   //a general function to call the command flag for issuing all other commands   
//  Serial.write(0xFE);
//}
//Ends References=========================
//===================================================
