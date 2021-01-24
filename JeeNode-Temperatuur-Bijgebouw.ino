#include <JeeLib.h>
#include <avr/sleep.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <dht.h>

#define DEBUG		1	// set to 1 to display each loop() run and PIR trigger

#define SEND_MSG_EVERY	38
//#define SEND_MSG_EVERY	22		// Watchdog is a timerTick on a avg 8,0 sec timebase
															// SEND_MSG_EVERY=8	-> +- 1min
															// SEND_MSG_EVERY=14 -> +- 2min
															// SEND_MSG_EVERY=23 -> +- 3min
															// SEND_MSG_EVERY=30 -> +- 4min
															// SEND_MSG_EVERY=38 -> +- 5min

#define SEND_MSG_BATT_EVERY	90	 // Measure battery voltage every N messages
																	// MEASURE_EVERY=90 -> +- 4 hour
#define SMOOTH					3		 // smoothing factor used for running averages

#define NODE_ID				 4				 // NodeId of this JeeNode
#define GROUP_ID				5				 // GroupId of this JeeNode

volatile unsigned char sendMsgTimer = SEND_MSG_EVERY;
volatile unsigned char sendMsgBatteryLevelTimer = SEND_MSG_BATT_EVERY;

//JeeNode message
struct payload {
	 byte batteryLevel;				 //getVcc 1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250
	 bool lobat;								// supply voltage dropped under 3.1V: 0..1
	 int oneWireTemp1;
	 int oneWireTemp2;
	 int dht22Temp;
	 int dht22Humi;
} payload;

volatile bool sendMsg = false;
volatile bool timerTick = false;
volatile bool adcDone = false;

//Pins JeeNode ports
#define ONE_WIRE_BUS 6 //P3.DIO
#define DHT22_PIN		7 //P4.DIO
#define DHTTYPE DHT22	 // DHT 22	(AM2302), AM2321

#ifdef ONE_WIRE_BUS
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature oneWireTemp(&oneWire);
#endif

#ifdef DHT22_PIN
dht DHT;
#endif

ISR(WDT_vect) { 
	 Sleepy::watchdogEvent();
	 if(!timerTick) { timerTick = true; }
}

ISR(ADC_vect) { adcDone = true; }

static byte batteryLevelRead (byte count =4) {
	 set_sleep_mode(SLEEP_MODE_ADC);
	 ADMUX = bit(REFS0) | 14; // use VCC and internal bandgap
	 bitSet(ADCSRA, ADIE);
	 while (count-- > 0) {
			adcDone = false;
			while (!adcDone) sleep_mode();
	 }
	 bitClear(ADCSRA, ADIE);	
	 // convert ADC readings to fit in one byte, i.e. 20 mV steps:
	 //	1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250
	 return (55U * 1023U) / (ADC + 1) - 50;
}

// readout all the sensors and other values
static void doMeasure() {
	 float h, t;
		
	 payload.lobat = rf12_lowbat();

#ifdef DHT22_PIN	 
	 // Reading temperature or humidity takes about 250 milliseconds!
	 // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
	 // Read temperature as Celsius (the default)
#if DEBUG
	 Serial.println();
	 Serial.print("DHT22 Temp:");
#endif
	 int chk = DHT.read22(DHT22_PIN);
	 
	 // Check if any reads failed and exit early (to try again).
	 if (chk == DHTLIB_OK) {
			payload.dht22Humi = DHT.humidity + 0.5;
			payload.dht22Temp = 10 * DHT.temperature + 0.5;
#if DEBUG
			// DISPLAY DATA
			Serial.print(payload.dht22Temp, 1);
			Serial.print(", Humi:");
			Serial.println(payload.dht22Humi, 1);
#endif			
	 } else {
			payload.dht22Temp = 9999; //Error
			payload.dht22Humi = 9999; //Error
#if DEBUG
			Serial.println("Failed to read from DHT sensor!");
#endif
	 }
#endif //DHT22_PIN

#ifdef ONE_WIRE_BUS	 
	 //Uitlezen oneWire temperatuur sensoren
	 oneWireTemp.requestTemperatures(); // Send the command to get temperatures
	 
	 t = oneWireTemp.getTempCByIndex(0);
	 payload.oneWireTemp1 = 10 * t + 0.5;
#if DEBUG
	 Serial.print("1-Wire Temp1:");
	 Serial.println(payload.oneWireTemp1, 1);
#endif

	 t = oneWireTemp.getTempCByIndex(1);
	 payload.oneWireTemp2 = 10 * t + 0.5;
#if DEBUG
	 Serial.print("1-Wire Temp2:");
	 Serial.println(payload.oneWireTemp2, 1);
#endif
	 
	 oneWire.depower();
#endif //ONE_WIRE_BUS
	 
}


void setup() {
#if DEBUG
	 Serial.begin(57600);
	 Serial.println("\n[Temperatuur bijgebouw]");
#endif

	 rf12_initialize(NODE_ID, RF12_868MHZ, GROUP_ID);
	 
#ifdef ONE_WIRE_BUS
	 // Start up the library
	 oneWireTemp.begin();
#endif //ONE_WIRE_BUS
	
	 Sleepy::watchdogInterrupts(6); //Start the watchdog timer for time base
}

void loop() {

	 if (timerTick) 
	 { // There has ben a Watchdog interrupt for time measurement (also possible wakeup by an interrupt)
			timerTick = false;

			sendMsgTimer++;
			
			if (sendMsgTimer >= SEND_MSG_EVERY) {
				 sendMsgTimer = 0;
				 
				 doMeasure();
				
				 sendMsgBatteryLevelTimer++;
				 if (sendMsgBatteryLevelTimer >= SEND_MSG_BATT_EVERY) {
					 sendMsgBatteryLevelTimer = 0;
					 payload.batteryLevel = batteryLevelRead();
				 }
				 sendMsg = true;
			}
	 }
	 
	 if (sendMsg) 
	 {
			sendMsg = false;
			
			//payload.nrOfPulses = nrOfPulses;
			
			rf12_sleep(RF12_WAKEUP);
			while (!rf12_canSend())
			rf12_recvDone();
			rf12_sendStart(0, &payload, sizeof payload);
			rf12_sendWait(1);
			rf12_sleep(RF12_SLEEP);
	 }
	 Sleepy::watchdogInterrupts(9);		// Start the watchdog timer for timerTick 6=5,2sec 9=8,31sec
	 Sleepy::powerDown();
}

