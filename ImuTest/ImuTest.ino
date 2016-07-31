#include <Wire.h>

#include <Adafruit_Sensors/Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U/Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303DLHC/Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_Unified/Adafruit_BMP085_U.h>

static sensors_event_t sensors_event;
static float temperature;

#define GRDY 2
Adafruit_L3GD20_Unified       gyro = Adafruit_L3GD20_Unified(&Wire, 30303);
static volatile bool gyroDataReady = false;

#define LIN1 3
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
static volatile bool accelDataReady = false;

#define LRDY 19
Adafruit_LSM303_Mag_Unified     mag = Adafruit_LSM303_Mag_Unified(30302);
static volatile bool magDataReady = false;
static volatile bool magDataRead = false;

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(30304);

#define TRACE
#define INTERRUPTS
//#define POLLINGINTERRUPTS

void setup()
{
	Serial.begin(250000);
	Serial.println("Starting up...");

	pinMode(GRDY, INPUT);
	attachInterrupt(digitalPinToInterrupt(GRDY), GyroDataReady, RISING);
	pinMode(LIN1, INPUT);
	attachInterrupt(digitalPinToInterrupt(LIN1), AccelDataReady, RISING);
	pinMode(LRDY, INPUT);
	attachInterrupt(digitalPinToInterrupt(LRDY), MagDataReady, RISING);      // DRDY/LRDY, Mag Data Ready


	gyro.enableAutoRange(true);
	if (!gyro.begin())
	{
		Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
	}
	//gyro.setOutputDataRate(GYRO_ODR_95);
	gyro.enableDRDYInterrupt(true);

	accel.enableAutoRange(true);
	if (!accel.begin())
	{
		Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
	}
	//accel.setAccelRate(LSM303_ACCEL_ODR_100);       // Default is 100 Hz
	accel.enableInt1DataReady(true);

	mag.enableAutoRange(true);
	if (!mag.begin()) {
		Serial.println(F("Ooops, no LSM303 detected (mag)... Check your wiring!"));
	}
	//mag.setMagRate(LSM303_MAGRATE_75);              // Default is 15 Hz
	// mag interrupt is on by default

	if (!bmp.begin())
	{
		Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
	}
}

void GyroDataReady()
{
	gyroDataReady = true;
}

void AccelDataReady()
{
	accelDataReady = true;
}

void MagDataReady()
{
	magDataReady = true;
}

void loop()
{
#if defined(INTERRUPTS)
	if (gyroDataReady)
#elif defined(POLLINGINTERRUPTS)
	if (digitalRead(GRDY))
#endif
	{
		if (gyro.getEvent(&sensors_event))
		{
#ifdef TRACE
			Serial.print("G");
#else
			sensors_vec_t vector = sensors_event.gyro;
			Serial.print("Gyro: "); Serial.print(vector.x); Serial.print(","); Serial.print(vector.y); Serial.print(","); Serial.println(vector.z);
#endif
			gyroDataReady = false;
		}
		else
		{
			// Don't reset the data ready flag, since we didn't actually clear the data.
			Serial.println("### Gyro Error ###");
		}
	}

#if defined(INTERRUPTS)
	if (accelDataReady)
#elif defined(POLLINGINTERRUPTS)
	if (digitalRead(LIN1))
#endif
	{
		if (accel.getEvent(&sensors_event))
		{
#ifdef TRACE
			Serial.print("A");
#else
			sensors_vec_t vector = sensors_event.acceleration;
			Serial.print("Accel: "); Serial.print(vector.x); Serial.print(","); Serial.print(vector.y); Serial.print(","); Serial.println(vector.z);
#endif
			accelDataReady = false;
		}
		else
		{
			// Don't reset the data ready flag, since we didn't actually clear the data.
			Serial.println("### Accel Error ###");
		}
	}

#if defined(INTERRUPTS)
	if (magDataReady)
#elif defined(POLLINGINTERRUPTS)
	if (digitalRead(LRDY) && !magDataRead)
#endif
	{
		if (mag.getEvent(&sensors_event))
		{
#ifdef TRACE
			Serial.print("M");
#else
			sensors_vec_t vector = sensors_event.magnetic;
			Serial.print("Mag: "); Serial.print(vector.x); Serial.print(","); Serial.print(vector.y); Serial.print(","); Serial.println(vector.z);
#endif
			magDataReady = false;
#if defined(POLLINGINTERRUPTS)
			magDataRead = true;
#endif
		}
		else
		{
			// Don't reset the data ready flag, since we didn't actually clear the data.
			Serial.println("### Mag Error ###");
		}
	}
#if defined(POLLINGINTERRUPTS)
	else
	{
		magDataRead = false;
	}
#endif

#if defined(INTERRUPTS)
	if (false)
#elif defined(POLLINGINTERRUPTS)
	if (false)
#else
#endif
	{
		if (bmp.getTemperature(&temperature))
		{
#ifdef TRACE
			Serial.print("T");
#else

			Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" C");
#endif
		}
		else
		{
			Serial.println("### Temp Error ###");
		}

		if (bmp.getEvent(&sensors_event))
		{
#ifdef TRACE
			Serial.print("P");
#else
			Serial.print("Pressure: "); Serial.print(sensors_event.pressure); Serial.print(" hPa; ");
			Serial.print("Altitude: "); Serial.print(bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, sensors_event.pressure)); Serial.println(" m");
#endif
		}
		else
		{
			Serial.println("### Alt Error ###");
		}
	}
}


