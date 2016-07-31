#include <Wire.h>
#include <Adafruit_Sensors/Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <HAL/HAL.h>

#define GPS_Serial Serial2
static HAL::HAL hal = HAL::HAL(&Serial);

Adafruit_GPS GPS(&GPS_Serial);
Adafruit_10DOF                dof = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro = Adafruit_L3GD20_Unified(30303);
Adafruit_BMP085_Unified       bmp = Adafruit_BMP085_Unified(18001);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

void initSensors()
{
	//if (!accel.begin())
	//{
	//	/* There was a problem detecting the LSM303 ... check your connections */
	//	Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
	//	//while (1);
	//}
	//if (!mag.begin())
	//{
	//	/* There was a problem detecting the LSM303 ... check your connections */
	//	Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
	//	//while (1);
	//}
	//if (!bmp.begin())
	//{
	//	/* There was a problem detecting the BMP180 ... check your connections */
	//	Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
	//	//while (1);
	//}

	gyro.enableAutoRange(true);
	if (!gyro.begin())
	{
		Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
	}
	gyro.enableDRDYInterrupt(true);
	gyro.setOutputDataRate(GYRO_ODR_190);
}

void GINT()
{
	HAL::debug("GINT: %d", digitalRead(49));
}

void GRDY()
{
	HAL::debug("GRDY: %d", digitalRead(51));
}

void setup()
{
	hal.init();
	HAL::debug("Starting up...");

	initSensors();

	pinMode(49, INPUT); // GINT
	attachInterrupt(49, GINT, RISING);

	pinMode(51, INPUT); // GRDY
	attachInterrupt(51, GRDY, RISING);

	//GPS.begin(9600);

	//// uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
	//GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	//// uncomment this line to turn on only the "minimum recommended" data
	////GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
	//// For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
	//// the parser doesn't care about other sentences at this time

	//// Set the update rate
	//GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
	//// For the parsing code to work nicely and have time to sort thru the data, and
	//// print it out we don't suggest using anything higher than 1 Hz

	//// Request updates on antenna status, comment out to keep quiet
	//GPS.sendCommand(PGCMD_ANTENNA);
	//
	//delay(1000);
	//// Ask for firmware version
	//GPS.sendCommand(PMTK_Q_RELEASE);
}

static uint32_t lastIMUReading;

static float roll, pitch, heading, temperature, altitude;
static float gyrox, gyroy, gyroz;

static sensors_event_t accel_event;
static sensors_event_t mag_event;
static sensors_event_t bmp_event;
static sensors_vec_t   orientation;

sensors_event_t gyro_event;
void loop()
{
	//gyro.getEvent(&gyro_event);

	//gyrox = gyro_event.gyro.x;
	//gyroy = gyro_event.gyro.y;
	//gyroz = gyro_event.gyro.z;
	//HAL::debug(debug_IMU, VERBOSE, "Gyro: %f,%f,%f", gyrox, gyroy, gyroz);

	HAL::debug("Testing");

	/* Calculate pitch and roll from the raw accelerometer data */
	//accel.getEvent(&accel_event);
	//if (dof.accelGetOrientation(&accel_event, &orientation))
	//{
	//	/* 'orientation' should have valid .roll and .pitch fields */
	//	roll = orientation.roll;
	//	pitch = orientation.pitch;
	//}

	///* Calculate the heading using the magnetometer */
	//mag.getEvent(&mag_event);
	//if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
	//{
	//	/* 'orientation' should have valid .heading data now */
	//	heading = orientation.heading;
	//}

	///* Calculate the altitude using the barometric pressure sensor */
	//bmp.getEvent(&bmp_event);
	//if (bmp_event.pressure)
	//{
	//	/* Get ambient temperature in C */
	//	bmp.getTemperature(&temperature);
	//	/* Convert atmospheric pressure, SLP and temp to altitude    */
	//	altitude = bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature);
	//	/* Display the temperature */
	//}

	//HAL::debug(debug_IMU, VERBOSE, "Roll: %f; Pitch: %f; Heading: %f; Alt: %f m; Temp: %f C;", roll, pitch, heading, altitude, temperature);

	//GPS.read();

	// if a sentence is received, we can check the checksum, parse it...
	//if (GPS.newNMEAreceived()) {

	//	char* lastNMEA = GPS.lastNMEA();
	//	if (GPS.parse(lastNMEA))
	//	{
	//		// Don't be confused because this will print out some lines that look like dupes.
	//		// Only part of the line will change with each update.
	//		HAL::debug(debug_GPS, VERBOSE, "[%02d/%02d/20%02d %02d:%02d:%02d.%03d] %d-%d (%02d) %08.4f%c-%08.4f%c Speed(k): %f Angle: %f Altitude: %f",
	//			GPS.month, GPS.day, GPS.year,
	//			GPS.hour, GPS.minute, GPS.seconds, GPS.milliseconds,
	//			GPS.fix, GPS.fixquality,
	//			GPS.satellites,
	//			GPS.latitude, GPS.lat,
	//			GPS.longitude, GPS.lon,
	//			GPS.speed, GPS.angle, GPS.altitude);
	//	}
	//}
}
