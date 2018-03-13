/*
Updated to reflect the control of a stepper
*/

#include <OneWire.h>
#include <Stepper.h>

#define motorSteps 200; // The number of steps per revolution

const int dirPin = 5;

// Set the microstepping F = Full, H = Half, Q = Quarter, E = Eigth, S = Sixteenth
char Microstep = 'F';
const int MS1Pin = 2;
const int MS2Pin = 3;
const int MS3Pin = 4;

int dt = 10;
int step, pin_steps, digital_pin;
String input_string="";
boolean string_complete=false;

OneWire temp_sensors(4); //set pin 4 to be the temperature sensor 
byte addr1[8]={0x10,0xE8,0xB9,0xA0,0x2,0x8,0x0,0x5F};
byte addr2[8] = {0x10,0xC1,0xB4,0xA0,0x2,0x8,0x0,0x93};
byte OneWireData[12];


void setup()
{

  //set up serial port and input buffer
  Serial.begin(9600);
  input_string.reserve(200);
  //set up digital out pins
  int digital_pin;
  for (digital_pin=2; digital_pin<54; digital_pin++){
    pinMode(digital_pin, OUTPUT);
    digitalWrite(digital_pin, LOW);    
  }
  #if defined(Microstep)
      switch (Microstep[0]){
      case 'F': {break;}
      case 'H': {digitalWrite(MS1Pin, HIGH); break;}
      case 'Q': {digitalWrite(MS2Pin, HIGH); break;}
      case 'E': {digitalWrite(MS1Pin, HIGH); digitalWrite(MS2Pin, HIGH); break;}
      case 'S': {digitalWrite(MS1Pin, HIGH); digitalWrite(MS2Pin, HIGH); digitalWrite(MS3Pin, HIGH); break;}
      default: // This will leave a Full Steps
      }
  #endif
}

void measure_analog(){
  if (input_string.length()>11)
  {
    int iter;
    float mean=0;
    float var_sq=0;
    float val; 
    int analog_pin = input_string.substring(1,3).toInt();
    int n_measurements = input_string.substring(3,7).toInt();
    int tmpdt = input_string.substring(7,11).toInt();
    for (iter=0; iter<n_measurements; iter++){
      val=(float)analogRead(analog_pin);
      mean += val;
      var_sq += val*val;
      delay(tmpdt);
    }
    mean /= (float)n_measurements;
    var_sq /= (float)n_measurements;
    var_sq -= mean*mean;
    Serial.print("A\t");
    Serial.print(analog_pin);
    Serial.print('\t');
    Serial.print(mean);
    Serial.print('\t');
    Serial.print(var_sq);
    Serial.print('\t');
    Serial.print(n_measurements);
    Serial.print('\t');
    Serial.println(tmpdt);
  }else{
    Serial.print("error: measure_analog() received messed up command: "); 
    Serial.println(input_string);
  }
}

void switch_digital(){
  digital_pin = input_string.substring(1,3).toInt();
  Serial.println(digital_pin);
  pin_steps = input_string.substring(3,8).toInt();
  Serial.println(pin_steps);
  if (pin_steps>0){
    stepping_function();
  }else if (pin_steps=0){
    digitalWrite(digital_pin,LOW);
  }else{
    Serial.print("error: switch_digital() received bad pin state: ");
    Serial.println(input_string);
  }
  Serial.print("D\t");
  Serial.print(digital_pin);
  Serial.print('\t');
  Serial.println(pin_steps);
}

void stepping_function(){
  for(step = 0; step < pin_steps; step++){
    digitalWrite(digital_pin, HIGH);
    delay(10);
    Serial.println("Step");
    digitalWrite(digital_pin, LOW);
    delay(10);
  }
}

void loop()
{
  if (string_complete){
    switch (input_string[0]){
    case 'A': {measure_analog(); break;}
    case 'D': {switch_digital(); break;} 
    case 'C': {start_temperature_conversion(); break;}
    case 'T': {read_temperature(); break;}
    default: {Serial.println("error: unknown command"); break;}
    }  
    input_string="";
    string_complete=false;    
  }
  else
  {
    // delay a few ms for stability
    delay(dt);
  }
}


/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    input_string += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      string_complete = true;
      //break;
    }
  }
}

void start_temperature_conversion(){
    temp_sensors.reset();
    temp_sensors.select(addr1);
    temp_sensors.write(0x44,0);          // start conversion, with parasite power off
    delay(10);

    temp_sensors.reset();
    temp_sensors.select(addr2);
    temp_sensors.write(0x44,0);          // start conversion, with parasite power off
}

void read_temperature(){
    float temp1,temp2;
    int i;
    byte present = 0;
    present = temp_sensors.reset();
    if (present){
	temp_sensors.select(addr1);    
	temp_sensors.write(0xBE);
	for ( i = 0; i < 9; i++) {   // we need 9 bytes of data
	    OneWireData[i] = temp_sensors.read();
	}
	temp1 =  ((OneWireData[1] << 8) + OneWireData[0] ) * 0.5;  // 12Bit = 0,0625 C per Bit
    }else{temp1=0;}

    present = temp_sensors.reset();
    if (present){
	temp_sensors.select(addr2);    
	temp_sensors.write(0xBE);
	for ( i = 0; i < 9; i++) {   // we need 9 bytes of data
	    OneWireData[i] = temp_sensors.read();
	}
	temp2 =  ((OneWireData[1] << 8) + OneWireData[0] ) * 0.5;  // 12Bit = 0,0625 C per Bit
    }else{
	temp2=0;
    }
    Serial.print("T\t");
    Serial.print(temp1);
    Serial.print('\t');
    Serial.println(temp2);
}
