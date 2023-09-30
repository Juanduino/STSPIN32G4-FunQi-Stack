#include <Arduino.h>
#include "../lib/Arduino-FOC/src/SimpleFOC.h"
#include "../lib/Arduino-FOC-drivers/src/SimpleFOCDrivers.h"
#include "../lib/Arduino-FOC-drivers/src/encoders/MT6835_STM32G4/MagneticSensorMT6835_STM32G4.h"
#include "../lib/TrapezoidalPlanner/TrapezoidalPlanner.h"


//#include "./LL_ADC_SETUP.h"
#include "./stspin32g4_hw.h"
#include "./funqi_hw.h"
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "arm_math.h"



#define PIN PIN_NEOPIX
#define NUMPIXELS 1
#define pi 3.1415926535897932384626433f

float converted_cordic_sine = 0.0f;
float converted_cordic_cosine = 0.0f;

int VM_Voltage = 0;  // variable to store the value read
int ADC12_IN2 = 0;
int ADC1_IN3 = 0;

float readMT6835 = 0.0f;
//target variable
float target_velocity = 0;
float target_I = 5;
//NEO_PIXEL
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// I2C 
int I2C_Internal_Adr = 71; //
TwoWire myWire2(INTERNAL_I2C3_SDA, INTERNAL_I2C3_SCL);

StepperMotor motor = StepperMotor(50);
StepperDriver8PWM driver = StepperDriver8PWM(PE9, PE8, PE11, PE10, PE13, PE12, PA14, PB0);

//MT6835 on STM32G4  
MagneticSensorMT6835_STM32G4 sensor = MagneticSensorMT6835_STM32G4(PD2);

Commander commander  = Commander(SerialUSB);

uint32_t ADC_Value_combined = 0;
uint16_t sample1, sample2;

//uint32_t buff_BIN_angles[700];
//*********************************************************************************************************************

//void doTarget(char* cmd) { commander.scalar(&target_velocity, cmd); }
//void doTargetI(char* cmd) { commander.scalar(&target_I, cmd); }

TrapezoidalPlanner planner(10);

void doPlanner(char *cmd){
  planner.doTrapezoidalPlannerCommand(cmd);
}

void MPlanner(char *cmd){
  planner.doMCommand(cmd);
}

void Setup_Driver(){

     myWire2.begin();
     Wire.setClock(1000000UL);
     Serial.println("Setup driver to 10v!");
     
  myWire2.beginTransmission(I2C_Internal_Adr); // Begin transmission to the Sensor 
  //Ask the particular registers for data
  myWire2.write(0x0B);
  myWire2.write(0xF0);
  myWire2.endTransmission();
  myWire2.requestFrom(I2C_Internal_Adr,1); // Request the transmitted two bytes from the two registers
  
  while(myWire2.available())    // slave may send less than requested
  { 
    uint8_t a = myWire2.read(); // receive a byte as character
    Serial.println(a, HEX);         // print the character
    Serial.println(a, DEC);
    Serial.println(a, BIN);

  }
  
  myWire2.beginTransmission(I2C_Internal_Adr);
  myWire2.write(0x01);
  myWire2.write(0x01);
  myWire2.endTransmission();
  myWire2.requestFrom(I2C_Internal_Adr,1); // Request the transmitted two bytes from the two registers
  
  while(myWire2.available())    // slave may send less than requested
  { 
    uint8_t b = myWire2.read(); // receive a byte as character
    Serial.println(b, HEX);         // print the character
    Serial.println(b, DEC);
    Serial.println(b, BIN);

  }

  myWire2.beginTransmission(I2C_Internal_Adr);
  myWire2.write(0x0B);
  myWire2.write(0x00);
  myWire2.endTransmission();

  myWire2.requestFrom(I2C_Internal_Adr,1); // Request the transmitted two bytes from the two registers
  
  while(myWire2.available())    // slave may send less than requested
  { 
    uint8_t c = myWire2.read(); // receive a byte as character
    Serial.println(c, HEX);         // print the character
    Serial.println(c, DEC);
    Serial.println(c, BIN);

  }

  delay(30);
  myWire2.beginTransmission(I2C_Internal_Adr);
  myWire2.write(0x01);
  myWire2.endTransmission(); // Ends the transmission and transmits the data from the two registers
  
  myWire2.requestFrom(I2C_Internal_Adr,1); // Request the transmitted two bytes from the two registers
  
  while(myWire2.available())    // slave may send less than requested
  { 
    uint8_t d = myWire2.read(); // receive a byte as character
    Serial.println(d, HEX);         // print the character
    Serial.println(d, DEC);
    Serial.println(d, BIN);

  }

  Serial.println("Driver @10v OK!");
      
}

void setup() {

  SerialUSB.begin(115200);
  Pins_State();
  myWire2.begin();
  Wire.setClock(1000000UL);
  // put your setup code here, to run once:
 
 

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.setBrightness(14); // Set BRIGHTNESS to about 1/5 (max = 255)

  // The first NeoPixel in a strand is #0, second is 1, all the way up
  // to the count of pixels minus one.
  for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    pixels.setPixelColor(i, pixels.Color(0, 100, 0));
    pixels.show();   // Send the updated pixel colors to the hardware.

  }
  
  
 
  //commander.add('T', doTarget, "target velocity");

   // power supply voltage [V]
  driver.voltage_power_supply = 24;
  // Max DC voltage allowed - default voltage_power_supply

  driver.voltage_limit = 17;

  motor.foc_modulation = FOCModulationType::SinePWM;

  // aligning voltage [V]
  motor.voltage_sensor_align = 2; // default 3V   

  // motor phase resistance [Ohms]
  motor.phase_resistance = 3.0; // Ohms - default not set

  // motor phase inductance [mH]
  motor.phase_inductance = 0.005; // [H] - default not set
  

  
  motor.controller = MotionControlType::angle; // default MotionControlType::velocity_openloop

  // controller configuration based on the control type 
  // velocity PID controller parameters
  // default P=0.5 I = 10 D = 0
  motor.PID_velocity.P = 0.15;
  motor.PID_velocity.I = 4;
  motor.PID_velocity.D = 0.0008;

  motor.P_angle.P = 1.05;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 2000;

  motor.velocity_limit = 50;
  // setting the limits
  // either voltage
  //motor.voltage_limit = 6; // Volts - default driver.voltage_limit
  // of current 
  motor.current_limit = 1.7; // Amps - default 0.2Amps
  motor.motion_downsample = 10; // - times (default 0 - disabled)

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best. 
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.05;

  CORDIC_Config_LL();

  SimpleFOCDebug::enable(&SerialUSB);

  //init sensor
  //sensor.init();
  
  //link motor and sensor
  motor.linkSensor(&sensor);
  //driver init
  driver.init();
  driver.setPwm(0,0);
  // link the motor to the driver
  motor.linkDriver(&driver);

  //  GCode move Gxx, GVxx, or GAxx - Example: G30 moves to position in rads. 
  //  GV10 sets velocity to 10 rads/s. GA5 sets acceleration to 5 rads/s/s.");
  planner.linkMotor(&motor);
  commander.add('G', doPlanner, "Motion Planner");
  commander.add('M', MPlanner, "Motion Planner");
  

  delay(100);
   
  motor.useMonitoring(SerialUSB);
  motor.init();

  Setup_Driver(); // Setup BUCK conveter to 10v (default 8v)
  

  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 50000;




  myWire2.beginTransmission(I2C_Internal_Adr);
  myWire2.write(0x09);
  myWire2.write(0xFF);
  myWire2.endTransmission();
  myWire2.requestFrom(I2C_Internal_Adr,1); // Request the transmitted two bytes from the two registers
  delay(300);
  while(myWire2.available())    // slave may send less than requested
  { 
    uint8_t b = myWire2.read(); // receive a byte as character
    SerialUSB.println(b, HEX);         // print the character
    SerialUSB.println(b, DEC);
    SerialUSB.println(b, BIN);

  }

  delay(300);

  myWire2.beginTransmission(I2C_Internal_Adr);
  myWire2.write(0x80);
  myWire2.endTransmission();
  myWire2.requestFrom(I2C_Internal_Adr,1); // Request the transmitted two bytes from the two registers
  delay(300);
  while(myWire2.available())    // slave may send less than requested
  { 
    uint8_t b = myWire2.read(); // receive a byte as character
    SerialUSB.println(b, HEX);         // print the character
    SerialUSB.println(b, DEC);
    SerialUSB.println(b, BIN);

  }

  /*

 uint8_t write_CAL_Freq = 0;
 write_CAL_Freq = sensor.writeMT6835_CAL_FREQ();  
 SerialUSB.println("MT6835 Write CAL Freq. ->");
 SerialUSB.println(write_CAL_Freq, BIN);

 delay(2000);


  uint8_t write_ABZ1 = 0;
 write_ABZ1 = sensor.setMT6835_ABZ1();  
 SerialUSB.println("MT6835 Write CAL Freq. ->");
 SerialUSB.println(write_ABZ1, BIN);

 delay(2000);

  uint8_t write_ABZ2 = 0;
 write_ABZ2 = sensor.setMT6835_ABZ2();  
 SerialUSB.println("MT6835 Write CAL Freq. ->");
 SerialUSB.println(write_ABZ2, BIN);

 delay(2000);


 if (sensor.writeEEPROM() == true){
    SerialUSB.println("writeEEPROM ACK OK!");
  } else {SerialUSB.println("writeEEPROM ACK failed!");}
  
  delay(7000);
  SerialUSB.println("writeEEPROM Power Cycle!!");


delay(2000);

 uint16_t readOut_ABZ = 0;
 readOut_ABZ = sensor.readMT6835_ABZ();  
 SerialUSB.println("MT6835 ABZ registers (0x007 & 0x008) ->");
 SerialUSB.println(readOut_ABZ, BIN);

 delay(500);


 uint8_t readOut_CAL_Freq = 0;
 readOut_CAL_Freq = sensor.readMT6835_CAL_FREQ();  
 SerialUSB.println("MT6835 CAL Freq. ->");
 SerialUSB.println(readOut_CAL_Freq, BIN);
  delay(2000);


  for (int i=0;i<700;i++)
{

  uint32_t rawbits = 0;
  rawbits = sensor.getRaw21bitMT6835();
  SerialUSB.print("RAW ->  ");
 SerialUSB.println(rawbits, BIN);

}


 delay(8000);

 
  //sensor.init();

 delay(1000);
  
  SerialUSB.println("Setting MT6835 Zero Position ->");
  motor.setPhaseVoltage(motor.voltage_sensor_align, 0,  _3PI_2);
  // wait for it to settle
  delay(2000);
  if (sensor.setZeroFromCurrentPosition() == true){
    SerialUSB.println("Zero Position ACK OK!");
  } else {SerialUSB.println("Zero Position ACK failed!");}
  
  delay(1000);
  // Read out latched angle
  for (int i=0;i<3;i++)
{
  readMT6835 = sensor.getCurrentAngle();
  Serial.println(readMT6835, 8);
}
 
 */
 
    set_QuadEncoderTimer();
    
    
    TIM2_init();
    SerialUSB.println("TIM2 init! DOne");

    set_ADC();
  //TIM15_Init();

   delay(100);
  
  
    SerialUSB.println("Motor initFOC!");

  
 
// CALIBRATION SPEED 4.2 RAD/S WHEN VELOCITY_OPENLOOP IS USED.
  // motor.target = 4.2;  

  /*

    for (int i=0;i<700;i++)
{


  

  uint16_t adc_value = return_adc();
  uint16_t adc_value1 = return_adc1();

  sensor.update();
   float electricalAngle_debug = motor.electricalAngle();
   
  SerialUSB.print("electricalAngle_debug->");
  SerialUSB.print(electricalAngle_debug);
  SerialUSB.print("  ");


  SerialUSB.print("Sample1 -> ");
  SerialUSB.println(adc_value);
  SerialUSB.print("Sample 2  -> ");
  SerialUSB.println(adc_value1);

   float current_angle = sensor.getAngle();
  SerialUSB.print("Current Angle ->  ");
  SerialUSB.println(current_angle, 8);
  delay(10);



  uint16_t TIM3_value = read_TIM3();

  Serial.print(i);
  Serial.print(",");
  Serial.print(TIM3_value, BIN);
  Serial.print(",");
  Serial.println(TIM3_value / (float)65536 * _2PI, 7);


  
  uint32_t timer2 = TIM2 -> CCR1;
  float get_angle_for = sensor.getRaw16bitTIM3();
  SerialUSB.print("GET RAW ANGLE - >");
  SerialUSB.print(get_angle_for, 5);
  SerialUSB.print("    TIMER2 CCR1 - >");
  SerialUSB.println(timer2, BIN);

  delay(10);

}
  */




motor.initFOC();

motor.setPhaseVoltage(0, 0, 0);

delay(5000);

/*
//CALIBRATE PIN HIGH WHEN MT6835 BREAKOUT_BOARD IS USED IN ABZ DIREECTION.
pinMode(PA5, OUTPUT); 
digitalWrite(PA5, HIGH);
 */

pinMode(PA5, OUTPUT); 
digitalWrite(PA5, LOW);

}


long start_ticks = 0;
long stop_ticks = 0;
float elapsed_ticks = 0.0f;

int32_t float_to_q312(float input) {
    int32_t q31 = (int32_t) (input * 2147483648.0f);
    q31 = (q31 > 0) ? (q31 << 1) : (-q31 << 1);
    return q31;
}



int32_t q31_value2;
float32_t value_f32_sine2;
float32_t value_f32_cosine2;
q31_t cordic_cosine2;
q31_t cordic_sine2;



float angle2 = 0;

float wrap_to_12(float x) {
    while (x > 1.0f) {
        x -= 2.0f;
    }
    while (x < -1.0f) {
        x += 2.0f;


    }
    return x;
}

int counter = 0;



void loop() {
/*
  int count = 0;
 start_ticks = micros(); 
 for (int i=0;i<10000;i++)
{
 

  //motor.PID_velocity.I = target_I;
  if (counter > 1000){

  bool isCountingUpward = TIM3->CR1 & 0x0010 ? false : true;
  SerialUSB.print("isCountingUpward - >  ");
  SerialUSB.println(isCountingUpward);
   uint32_t timer2 = TIM2 -> CCR1;
  SerialUSB.print("TIMER2 CCR1 - >  ");
  SerialUSB.println(timer2);

  float speed = (4 * pi / 65536) * 168000000 / ((float)timer2 - 1.0f);
  SerialUSB.print("Rad/s - >  ");
    if (!isCountingUpward) {
        speed = -speed;}

  SerialUSB.println(speed);
  counter =0;

  float senor_test_vel = sensor.getVelocity(); 
   SerialUSB.print("Vel - >  ");
  SerialUSB.println(senor_test_vel);

  }

 
 //if (counter > 4){

   
   uint16_t adc_value3 = return_adc();
  uint16_t adc_value4 = return_adc1();


  SerialUSB.print("ADC1:");
  SerialUSB.print(adc_value3);
  SerialUSB.print(",");
  SerialUSB.print("ADC2:");
  SerialUSB.print(adc_value4);  
  SerialUSB.println();

//counter = 0;

 //}


 int count = 0;
 start_ticks = micros(); 
 for (int i=0;i<10000;i++)
{
   


  ********************************  CORDIC TEST  ********************************
 */

  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  //motor.monitor();

  commander.run();
  planner.runPlannerOnTick();
  
  
  //count++;



/*

int count = 0;

//start_ticks = micros();

for (angle2 = 0; angle2 <= pi*2; angle2 += 0.01, count++) {

    
  

    // Convert to radians
   

    // Convert to q1.31 fixed-point format
    uint32_t q1_31_angle = (uint32_t)(angle2 * (1UL << 31) / (1.0f * PI));


CORDIC->WDATA = q1_31_angle;
cordic_sine2 = CORDIC->RDATA;
cordic_cosine2 = CORDIC->RDATA;

value_f32_sine2 = (float32_t)cordic_sine2/(float32_t)0x80000000;
value_f32_cosine2 = (float32_t)cordic_cosine2/(float32_t)0x80000000;


value_f32_sine2 = wrap_to_12(value_f32_sine2);
value_f32_cosine2 = wrap_to_12(value_f32_cosine2);

float32_t alpha;
float32_t beta;

arm_inv_park_f32 (0.4f, 0.2f, &alpha, &beta, value_f32_sine2, value_f32_cosine2);


  Serial.print("sine:");
  Serial.print(alpha, 8);
  Serial.print(",");
  Serial.print("cosine:");
  Serial.print(beta, 8);  
  Serial.println();

 delay(50);

}


 // count++;

//}



stop_ticks = micros(); 
elapsed_ticks = stop_ticks-start_ticks;
Serial.print("time per iterasion:   ");
Serial.println(elapsed_ticks / count, 4);

*/
   
}

/*
extern DMA_HandleTypeDef hdma_adc1;
extern "C" void DMA1_Channel2_IRQHandler(void)
{HAL_DMA_IRQHandler(&hdma_adc1);}
*/

