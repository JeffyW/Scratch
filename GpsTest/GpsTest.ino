#include <HAL/HAL.h>
#include <Adafruit_GPS/Adafruit_GPS.h>

#define GPS_Serial Serial2
Adafruit_GPS GPS(&GPS_Serial);

void setup()
{
	GPS.begin(9600);

	// uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	// uncomment this line to turn on only the "minimum recommended" data
	//GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
	// For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
	// the parser doesn't care about other sentences at this time

	// Set the update rate
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
	// For the parsing code to work nicely and have time to sort thru the data, and
	// print it out we don't suggest using anything higher than 1 Hz

	// Request updates on antenna status, comment out to keep quiet
	GPS.sendCommand(PGCMD_ANTENNA);
	
	delay(1000);
	// Ask for firmware version
	GPS.sendCommand(PMTK_Q_RELEASE);
}

void loop()
{
	GPS.read();

	// if a sentence is received, we can check the checksum, parse it...
	if (GPS.newNMEAreceived()) {

		char* lastNMEA = GPS.lastNMEA();
		if (GPS.parse(lastNMEA))
		{
			// Don't be confused because this will print out some lines that look like dupes.
			// Only part of the line will change with each update.
			HAL::debug(debug_GPS, VERBOSE, "[%02d/%02d/20%02d %02d:%02d:%02d.%03d] %d-%d (%02d) %08.4f%c-%08.4f%c Speed(k): %f Angle: %f Altitude: %f",
				GPS.month, GPS.day, GPS.year,
				GPS.hour, GPS.minute, GPS.seconds, GPS.milliseconds,
				GPS.fix, GPS.fixquality,
				GPS.satellites,
				GPS.latitude, GPS.lat,
				GPS.longitude, GPS.lon,
				GPS.speed, GPS.angle, GPS.altitude);
		}
	}
}
