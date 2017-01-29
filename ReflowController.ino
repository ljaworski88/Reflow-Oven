/* Reflow Oven Controller
 2016 Lukas Jaworski
 based on the Reflow Controller by Karl Pitrich
  v 0.7.2
*/ 

//-----------------------------------------------------------------------------
//Optional Defines
//-----------------------------------------------------------------------------

// #define AUTOPID true // autotune PID parameters
#define ZERO_X_CALIBRATION true

#ifdef ZERO_X_CALIBRATION
	#define CALIBRATION_CYCLES 12 //set how many calibration cyles are desired, the first two and last readings are thrown out due to inconsistant readings because of intialization and shutdown
#endif

//-----------------------------------------------------------------------------
//File Depencencies
//-----------------------------------------------------------------------------

#include "helpers.h"
#include <avr/io.h>
// #include <avr/eeprom.h>
// #include <EEPROM.h>
#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <TimerOne.h>
#include <Servo.h>
#ifdef AUTOPID
  #include <PID_AutoTune_v0.h>
#endif

// ----------------------------------------------------------------------------
// Hardware Configuration
// ----------------------------------------------------------------------------

// 1.8" TFT via SPI

#define LCD_CS   10
#define LCD_OC   A2
#define LCD_RST  A3
#define SD_CS	 A0


// Thermocouples

// Thermocouple via SPI-MAX31855
#define THERMOCOUPLE1_CS  5
#define THERMOCOUPLE2_CS  4 //include later

//  Heater Solid State Relays

#define PIN_HEATER_TOP   9 // SSR for the top heating elements- separate later
#define PIN_HEATER_BOTTOM 8 // SSR for the bottom heating elements

//fan relay &| servo 

#define PIN_FAN      A1 // SSR for the fan- This reflow oven has no fan
#define PIN_SERVO     7 // servo control signal

// Rotary Encoder

#define ROTARY_A 	A4
#define ROTARY_B	A5
#define SW_PUSH		2

// Zero Crossing Detector

#define PIN_ZX       3 // pin for zero crossing detector
#define INT_ZX       digitalPinToInterrupt(PIN_ZX) // interrupt for zero crossing detector

// Indicator Light

#define RED 6
#define GREEN 0

//-----------------------------------------------------------------------------
// Profiles
//-----------------------------------------------------------------------------

typedef struct reflowProfile { //a complete reflow profile in terms temperaturs and temperature rates
  double rampSoakRate;	//ramp to soak in degC/s
  double soakLowTemp;	//temp in degC to start soak
  double soakRate;		//gentle ramp during soak in degC/s
  double soakHighTemp;	//temp in degC to stop soak and ramp to peak
  double rampPeakRate;	//ramp to peak in degC/s
  double peakTemp;		//peak temp in degC
  double coolDownRate;	//cooling rate in degC/s
  double safeTemp;		//temp in degC where oven is cool enough to work with
} reflowProfile;

typedef struct PID_constants { //a collection of PID constants
  double Kp;
  double Ki;
  double Kd;
} PID_constants;

typedef struct PIDprofile { //a profile to adjust PID constants during reflow
  PID_constants rampSoak;
  PID_constants soak;
  PID_constants rampPeak;
  PID_constants coolDown;
} PIDprofile;

typedef struct currentParameters { //PID Input, Setpoint, and Output respectively
	double avgTemp;
	double setpoint;
	double PID_output;
} currentParameters;

//-----------------------------------------------------------------------------
//TYPEDEFS
//-----------------------------------------------------------------------------

typedef enum reflowState { //state machine
  None     = 0,
  Idle     = 1,
  
  RampToSoak = 10,
  Soak,
  RampUp,
  RampDown,
  CoolDown,

  Complete = 20,

  Tune = 30
} reflowState;

typedef enum errorCode { //handle all currently known error states of the reflow oven
	NoError = 0,
	UnderTemp,
	OverTemp,
	BadReadings,
	NoMAX,
	VccShort,
	GndShort,
	OpenCircuit
} errorCode;


typedef union MAX31855_t { // thermocouple temp & status data, directly from the MAX31855
  uint32_t	value;
  uint8_t	bytes[4];
/*
bits 31-18 hold the thermocouple data, 
bit 16 holds the fault bit (0 if no fault and 1 if there is one), this is not used at all in the code,
bits 15-4 hold the cold junction temperature, 
bits 2-0 explain the fault
*/   
} MAX31855_t;
 

typedef struct Thermocouple { // processed thermocouple data, status, and CS pin of the MAX31855
  double temperature;
  uint8_t stat;
  uint8_t chipSelect;
} Thermocouple;

typedef struct rollingAvg { //holds the last ten actual temperature readings used in claclulating the rolling average, and keeps track of which position in the array needs to be updated next
	double temps[10];
	uint8_t position = 0;
} rollingAvg;

typedef struct cyclCount {//the "cycle counter" is how the PID output is translated into heater on/off times
	uint8_t tensPlace;
	uint8_t thirds;
	uint8_t positionThirds;
	uint8_t positionTens;
} cyclCount;

//-------------------------------------------------------------------------------
//Global Variables
//-------------------------------------------------------------------------------

const uint8_t		uSecPerTick = 100; //sets the timer tick rate that is used throughout the program, if you want to change this go over the code very carefully, especially the ISRs
const double		uSecPerSec = 0.0001;//conversion factor for the timer 0.0001sec = 100 usec
const uint32_t		updateTime = 10000; //update the setpoints every 1s - the updateTime variable is tied to the 100usec counter

volatile bool       cycleFinished = true; //indicates if the current "PWM" cycle is finished
volatile bool		offCounter = false; //the state of the countdown timer to turn off the SSR.
volatile uint8_t	offTimerTicks = 0;
volatile uint32_t	timerTicks = 0; //100usec timer keeper (no overflow detection)
// volatile uint32_t	zeroXticks = 0; //counts the zero crossing events, used in PID control
uint32_t			lastTimerTicks = 0;
uint32_t            lastPIDticks = 0;//used to ensure that the PID computes every time a heating "cycle" is finnished
uint8_t				usecPerZX = 83;//8.3ms (or 83 ticks @ 100us/tick) between ZeroX for 60Hz mains, 
									//sub in 10 for 50Hz. 
									//This value will be updated if a calibration is performed.
Thermocouple		thermoInput1; //intialize thermocouple 1
// Thermocouple		thermoInput2; //intialize thermocouple 2
uint8_t				servoPos = 0;
Servo				doorServo;
const uint32_t 		thermoUpdateTime = 250;
uint32_t			lastThermoUpdate;
reflowProfile		curProfile = {1.1, 135.0, 0.6, 165.0, 1.3, 225.0, 2.0, 50.0}; //the current reflow profile (also the only)
PIDprofile			curPIDprofile = {{1.1, 0.5, 30},{1.0, 0.1, 35.0},{3.0, 0.5, 5.0},{7.0, 0.01, 2.0}}; /*{{1.1, 0.101, 17.0},{1.0, 0.1, 18.0},{1.1, 0.1, 17.0},{7.0, 0.01, 2.0}}; //Profile of PID constants */
currentParameters	curParameters; //the current temp, setpoint, and pid output
reflowState			curOvenState = None; //current phase of the reflow cycle
rollingAvg			curRollingAvg; //a collection of past temp readings used in the rolling average
volatile cyclCount	cycleCounter = {0, 0, 0, 0}; //used to control heater on/off timings
volatile errorCode	curError = NoError;

PID					tempPID(&(curParameters.avgTemp),&(curParameters.PID_output),&(curParameters.setpoint),curPIDprofile.rampSoak.Kp,curPIDprofile.rampSoak.Kp,curPIDprofile.rampSoak.Kp,DIRECT);

#ifdef ZERO_X_CALIBRATION //These additional variables will be needed only if a ZeroX calibration is performed.
	volatile uint16_t	calibrationCycle = 0;
	volatile uint32_t	lastCalibrationTicks = 0;
	bool				calibrationFinished = false;
	uint16_t			calibrationArray[CALIBRATION_CYCLES];
	double				calibrationAvg = 0.0;
#endif

//-----------------------------------------------------------------------------
//FUNCTIONS
//-----------------------------------------------------------------------------

void readThermocouple(Thermocouple* input, volatile errorCode& ErrorCode) { //function to read the data from the MAX31855 and handle any errors that may arrise
  MAX31855_t readSensor;

  // uint8_t lcdState = digitalRead(LCD_CS);
  // digitalWrite(LCD_CS, HIGH);
  SPI.beginTransaction(SPISettings(4000000,MSBFIRST,SPI_MODE0));
  digitalWrite(input->chipSelect, LOW);
  for (int8_t i = 3; i >= 0; i--){
    readSensor.bytes[i] = SPI.transfer(0);
  }
  digitalWrite(input->chipSelect, HIGH);
  SPI.endTransaction();
  if (readSensor.value == 0){ //if nothing but zeroes is read then the MAX31855 is not connected or there is something seriously wrong, error is thrown
    ErrorCode = NoMAX;
    return;
  }
  input->stat = readSensor.bytes[0] & 0b111;
  if (input->stat){//this is the error handling portion which reads the MAX31855 error codes and passes them along to the main program
    switch (input->stat) {
      case 0b001:
        ErrorCode = OpenCircuit;
        return;
      case 0b010:
        ErrorCode = GndShort;
        return;
      case 0b100:
        ErrorCode = VccShort;
        return;
    }
  }
  uint16_t value = (readSensor.value >> 18) & 0x3FFF; //shift to the correct alignment for the temp data  
  input->temperature = (double) value * 0.25;
  // digitalWrite(LCD_CS, lcdState);
}

void openDoor(void){ //currently a dummy function
	//use servo to open door fully
	digitalWrite(RED, HIGH);
	doorServo.write(180);
	
}

void closeDoor(void){//currently a dummy function
	//use servo to fully close door
	doorServo.write(0);
}

void indicateDone(bool errorFlag = false){//currently a dummy function
	//indicate that the reflow is complete (Use LED AND buzzer to indicate that reflow is complete)
	
}

void updateSetpoints(reflowState& ovenState, reflowProfile Profile, currentParameters& Params, PID& PID, PIDprofile PIDprofile){ //This function updates the setpoints of the reflow profile at the specified updateTime (which is counted in 100usec ticks)
	uint32_t timeSinceUpdate = timerTicks-lastTimerTicks;
	// Serial.print("timeSinceUpdate: ");
	// Serial.println(timeSinceUpdate);
	// Serial.print("timerTicks: ");
	// Serial.println(timerTicks);
	// Serial.print("lastTimerTicks: ");
	// Serial.println(lastTimerTicks);
	if (timeSinceUpdate>updateTime){
		// Serial.println("updating");
		if (ovenState>Idle && ovenState<RampDown){
			switch(ovenState) {
				case RampToSoak:
					Params.setpoint += Profile.rampSoakRate*double(timeSinceUpdate)*(uSecPerSec);
					if (Params.setpoint > Profile.soakLowTemp){
						ovenState = Soak;
						PID.SetTunings(curPIDprofile.soak.Kp,curPIDprofile.soak.Kp,curPIDprofile.soak.Kp);
					}
				break;
				
				case Soak:
					Params.setpoint += Profile.soakRate*double(timeSinceUpdate)*(uSecPerSec);
					if (Params.setpoint > Profile.soakHighTemp){
						ovenState = RampUp;
						PID.SetTunings(curPIDprofile.rampPeak.Kp,curPIDprofile.rampPeak.Kp,curPIDprofile.rampPeak.Kp);
					}
				break;
				
				case RampUp:
					Params.setpoint += Profile.rampPeakRate * double(timeSinceUpdate)*(uSecPerSec);
					if (Params.setpoint > (Profile.peakTemp - 5.0)){
						PID.SetMode(MANUAL);
						Params.PID_output = 0;
					}
					if (Params.setpoint > Profile.peakTemp){
						openDoor();
						ovenState = RampDown;
						PID.SetTunings(curPIDprofile.coolDown.Kp,curPIDprofile.coolDown.Kp,curPIDprofile.coolDown.Kp);
					}
				break;
			}
		}
		else if (ovenState>=RampDown && ovenState<Complete){
			switch(ovenState) {
				case RampDown:
					Params.setpoint -= Profile.coolDownRate*double(timeSinceUpdate)*(uSecPerSec);
					if (Params.setpoint < Profile.safeTemp){
						ovenState = CoolDown;
						PID.SetMode(MANUAL);
						Params.PID_output = 0;
					}
				break;
				
				case CoolDown:
					Params.setpoint = 0.0;
					ovenState = Complete;
				break;
			}
		}
		else {
			if(ovenState == Complete){
				//indicateDone();
				ovenState = Idle;
			}
		}
		
		lastTimerTicks = timerTicks;	
	}

}

void rollAverage(rollingAvg& rollAvg, currentParameters& Params, volatile errorCode& ErrorCode){//takes a new temperature reading and averages it with the previous 9 readings, this function pulls the temperatures as fast as the function is reached
	uint32_t timeSinceUpdate = timerTicks-lastThermoUpdate;
	static uint8_t errorTempCount = 0;
	double curReading;
	if (timeSinceUpdate>thermoUpdateTime){
		readThermocouple(&thermoInput1, ErrorCode);
		curReading = thermoInput1.temperature;
		if ((curReading - Params.avgTemp) > 10 || (curReading - Params.avgTemp) < -10){//this checks if an individual reading is more than 10C different than the average, indicating a bad reading (should be impossible to move that much in a couple ms)
			errorTempCount += 3;
			if (errorTempCount >= 15){//only throw an error if the bad readings get excessive
				ErrorCode = BadReadings;
				return;
			}
		}
		else if (errorTempCount > 0){
			errorTempCount --;
		}
		
		//the next few lines handle the averaging of the temperature *note that only one value is removed and added at a time to save caluclation cycles
		
		Params.avgTemp -= rollAvg.temps[rollAvg.position]/10.0; //removes the reading that was at this postion previousl (the average adjusted amount)
		rollAvg.temps[rollAvg.position] = curReading;
		Params.avgTemp += curReading/10.0; //adds in the current reading into the average
		rollAvg.position ++;
		if (rollAvg.position > 9){
			rollAvg.position = 0;
		}
		curReading = Params.avgTemp - Params.setpoint;//repurpose the curReading variable to see if the current average is too far from the setpoint
		if (curReading>50.0 || curReading<-50.0){
			if (curReading>10){
				ErrorCode = OverTemp;
				if (curOvenState>=RampDown && curOvenState<=Complete){
					ErrorCode = NoError;
				}
			}
			else{
				ErrorCode = UnderTemp;
			}
		}
		lastThermoUpdate = timerTicks;
	}
	
}

void pinSetup(){
	pinMode(THERMOCOUPLE1_CS, OUTPUT);
	digitalWrite(THERMOCOUPLE1_CS, HIGH);
	pinMode(THERMOCOUPLE2_CS, OUTPUT);
	digitalWrite(THERMOCOUPLE2_CS, HIGH);
	pinMode(PIN_HEATER_BOTTOM, OUTPUT);
	digitalWrite(PIN_HEATER_BOTTOM, LOW);
	pinMode(PIN_HEATER_TOP, OUTPUT);
	digitalWrite(PIN_HEATER_TOP, LOW);
	pinMode(PIN_FAN, OUTPUT);
	digitalWrite(PIN_FAN, LOW);
	pinMode(RED, OUTPUT);
	digitalWrite(RED, LOW);
	// pinMode(GREEN, OUTPUT);
	// digitalWrite(GREEN, LOW);
	pinMode(LCD_CS, OUTPUT);
	digitalWrite(LCD_CS, HIGH);
	pinMode(SD_CS, OUTPUT);
	digitalWrite(SD_CS, HIGH);
	pinMode(LCD_OC, OUTPUT);
	digitalWrite(LCD_OC, LOW);
	pinMode(LCD_RST, OUTPUT);
	digitalWrite(LCD_RST, LOW);
	pinMode(ROTARY_A, INPUT);
	pinMode(ROTARY_B, INPUT);
	pinMode(SW_PUSH, INPUT);
	doorServo.attach(PIN_SERVO);
}

double initalTemp(rollingAvg& rollAvg, volatile errorCode ErrorCode){//Gets an intial value for the oven temperature
	double averageTemp = 0;
	for (uint8_t i = 0; i <10; i++){
		readThermocouple(&thermoInput1, ErrorCode);
		rollAvg.temps[i] = thermoInput1.temperature;
		averageTemp += thermoInput1.temperature/10.0;
		delay(25);
	}
	lastThermoUpdate = timerTicks;
	return averageTemp;
}

void updatePID(PID& PID, currentParameters Params, volatile cyclCount& cycleSet, volatile bool& update){//updates the PID output and set up the next 1/2sec of heater function (which runns on a % of the 30 AC cycles that should happen in that time)
	//this code only responds to changes in 3% on time segments, so a change of at least 3% is needed to see an effect (i.e. 90%=91%=92%<93%)
	uint8_t intOutput;
	uint32_t now = millis();
	cli(); //interrupts are disabled here because they were interferring with the serial print function
	if(update){
        // PID.SetSampleTime(now-lastPIDticks);
        PID.Compute(); //the output is set from 0-33, so that each whole number change of the PID actually has an effect on heater on time
		intOutput = int(Params.PID_output)*3; //this makes it so the next part sees a number from 0-99%
		// Serial.println("Params.PID_output");
		// Serial.println(intOutput);
		// Serial.println("Params.setpoint");
		// Serial.println(Params.setpoint);
		// Serial.println("Params.avgTemp:");
		// Serial.println(Params.avgTemp);
		Serial.print(intOutput);//This serial data is feed into a python program to graph the datapoints
		Serial.print(",");
		Serial.print(Params.setpoint);
		Serial.print(",");
		Serial.println(Params.avgTemp);
		cycleSet.tensPlace = intOutput / 10; //the 30 cycles are divided into 3 10 subcycle segments, with the tens place indicating how many of each of the 3 subcycles will be on for (0%-9% = 0; 10%-19% = 1 ... 90%-99% = 9; and 100% does not exist technically but is functionally the same as a 99%)
		cycleSet.thirds = (intOutput % 10) / 3; //will turn on an extra cycle based on the ones place of the PID output. (0%-2% = 0 extra cycles; 3%-5% = 1 extra cycles ... 7%-9% = 3 extra cycles: this end up meaning that 9% and 10%; 19% and 20%; etc. are equivilent in terms of heater on time)
		cycleSet.positionThirds = 0;
		cycleSet.positionTens = 0;
		update = false;
		lastPIDticks = millis();        
    }
	sei();
}

void checkErrors(volatile errorCode ErrorCode){//This function checks for any reflow ending errors, shuts off the oven, and spits out an error code
    if (curOvenState != Idle || curOvenState != Complete){
		if (ErrorCode){
			cli();//interrupts are no longer need at this point so they are disabled
			digitalWrite(PIN_HEATER_BOTTOM, LOW); //Turns off the heating elements
			switch(ErrorCode){
				case UnderTemp:
					while(1){
						Serial.println("UT"); //Undertemp
						delay(1000);
					}
					break;
				case OverTemp:
					while(1){
						Serial.println("OT"); //Overtemp
						delay(1000);
					}
					break;
				case BadReadings:
					while(1){
						Serial.println("BR"); //Too many bad readings from the MAX31855
						delay(1000);
					}
					break;
				case NoMAX:
					while(1){
						Serial.println("NM"); //no MAX31855 detected
						delay(1000);
					}
					break;
				case VccShort:
					while(1){
						Serial.println("VS"); //MAX31855 Vcc short error
						delay(1000);
					}
					break;
				case GndShort:
					while(1){
						Serial.println("GS");//MAX31855 Gnd short error
						delay(1000);
					}
					break;
				case OpenCircuit:
					while(1){
						Serial.println("OC");//MAX31855 Open Circuit error
						delay(1000);
					}
					break;
			}
		}

	}
}

//-------------------------------------------------------------------------------
//ISRs
//-------------------------------------------------------------------------------

void timerISR(void){//The timer is used for the ZeroX calibration and to turn off the relay slightly before the zeroX event to ensure the triac stays off when it is supposed to
#ifdef ZERO_X_CALIBRATION 
    if(!calibrationFinished){
        timerTicks++;
	}
    else {
#endif
		if(offCounter && (offTimerTicks > usecPerZX - 10 )){//turns the relay off about 1ms before the start of an off cycle to ensure the triac stays off
		//currently the code will go through the offTimer code EVERY time the next cycle is supposed off, this is inefficent but does not cause a problem in the 16MHz Arduinos tested, slower clocked chips may encounter problems (this is untested though)
			digitalWrite(PIN_HEATER_BOTTOM, LOW);
			offCounter = false; //the off timer is turned off once it has completed its function.
			offTimerTicks = 0;
		}
		else if(offCounter){
			offTimerTicks++;
		}
	timerTicks++;
#ifdef ZERO_X_CALIBRATION
	}
#endif
	
}

//note: the zeroXingISR needs an external interrupt to run. If you cannot hook up a ZeroX detector (I recommend the H11AA1 for this) then use a 555 timer and set
// it to run at approximately your desired frequency. Without a true ZeroX detector this code won't run as well but if you are unsure about working with mains
//power it will keep you safer (still be careful around the relay).

void zeroXingISR(void){ //This ISR is involved in "PWMing" the heater elements of the reflow oven. Each "cycle" is based on 30 zeroXings (so it should last about 1/2sec)
#ifdef ZERO_X_CALIBRATION //This calibration will calculate the time between ZeroXings in terms of 100usec ticks. 60Hz should have 166.7 ticks on average and 50Hz 200.0
	if(!calibrationFinished){ //will only run during calibration otherwise ignored
		uint16_t timePerZX;//temporary variable to hold how long it takes between zero crossings
		timePerZX = timerTicks - lastCalibrationTicks;
		lastCalibrationTicks = timerTicks;
		calibrationArray[calibrationCycle] = timePerZX;
		calibrationCycle++;
	}
	else{
#endif
		if(!cycleFinished){
			if(cycleCounter.positionThirds < 3){
				if(cycleCounter.positionTens < cycleCounter.tensPlace){
					digitalWrite(PIN_HEATER_BOTTOM, HIGH);
					cycleCounter.positionTens ++;
				}
				else if(cycleCounter.positionTens < (cycleCounter.tensPlace + 1) && cycleCounter.positionThirds < cycleCounter.thirds){
					digitalWrite(PIN_HEATER_BOTTOM, HIGH);
					cycleCounter.positionTens ++;
				}
				else{
					digitalWrite(PIN_HEATER_BOTTOM, LOW);
					cycleCounter.positionTens ++;
				}
				if(cycleCounter.positionTens == 10){
					cycleCounter.positionThirds ++;
					cycleCounter.positionTens = 0;
				}
				if((cycleCounter.positionTens == cycleCounter.tensPlace) && (cycleCounter.positionThirds >= cycleCounter.thirds)){
				offCounter = true; //starts a countdown timer to turn off the SSR before the next zeroX
				}
			}
			
			if(cycleCounter.positionThirds == 3){
				cycleFinished = true; //indicates that the cycle is complete and is waiting for a new PID computation
				cycleCounter.positionThirds = 0;
				cycleCounter.positionTens = 0;
			}
		}
#ifdef ZERO_X_CALIBRATION
	}// zeroXticks++;
#endif
}

//-------------------------------------------------------------------------------
//Main Program
//-------------------------------------------------------------------------------

void setup(){
	delay(5000); //gives some time to turn on the serial monitor or reupload new code without starting the oven
	Serial.begin(115200); // Debugging Serial output/data output
	SPI.begin();
	
#ifdef ZERO_X_CALIBRATION
//This portion calculates how long a ZeroX event takes and uses this value in timing the SSR relay to turn off reliably (a triac will not turn off even if the gate voltage is no longer applied, until a ZeroX event takes place)
    pinMode(PIN_ZX, INPUT);
	attachInterrupt(INT_ZX, zeroXingISR, RISING);
    Timer1.initialize(uSecPerTick);
    Timer1.attachInterrupt(timerISR);
	
	//wait for the calibration to run through the required amount of cycles to get a good average
	while(calibrationCycle<CALIBRATION_CYCLES){
	}
	//turn off the calibration portions of the ISRs
	calibrationFinished = true;
    Timer1.detachInterrupt();

	for(uint8_t i=2;i<CALIBRATION_CYCLES;i++){
		calibrationAvg += (double)calibrationArray[i]/((double)CALIBRATION_CYCLES - 2.0); //the first 2 readings and the last reading in the array are garbage so throw them out
	}
	usecPerZX = int(calibrationAvg);
#endif
	thermoInput1.chipSelect = THERMOCOUPLE1_CS;
	pinSetup();
	closeDoor();
	curOvenState = RampToSoak;
	Timer1.initialize(uSecPerTick);
	Timer1.attachInterrupt(timerISR);
	curParameters.avgTemp = initalTemp(curRollingAvg, curError);
	curParameters.setpoint = curParameters.avgTemp;
	tempPID.SetOutputLimits(0, 33); // set output of heaters, since the heaters can only respont to 3% incraments the range here is set from 0-33 and is later multiplied by 3
	lastTimerTicks = timerTicks; //counts in 100usec ticks
    lastPIDticks = millis(); //counts in ms
	tempPID.SetMode(AUTOMATIC);
}

//-------------------------------------------------------------------------------

void loop(){
	// Serial.println(calibrationAvg);
	updateSetpoints(curOvenState, curProfile, curParameters, tempPID, curPIDprofile);
	rollAverage(curRollingAvg, curParameters, curError);
	checkErrors(curError);
	updatePID(tempPID, curParameters, cycleCounter, cycleFinished);
}