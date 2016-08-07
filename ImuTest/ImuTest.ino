#include <Wire.h>
#include <I2C-Master-Library/I2C.h>

#include <Adafruit_Sensors/Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U/Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303DLHC/Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_Unified/Adafruit_BMP085_U.h>

static sensors_vec_t vector;

#define GRDY 2
Adafruit_L3GD20_Unified       gyro = Adafruit_L3GD20_Unified(30303);
static volatile bool gyroDataReady = false;

#define LIN1 3
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
static volatile bool accelDataReady = false;

#define LRDY 19
Adafruit_LSM303_Mag_Unified     mag = Adafruit_LSM303_Mag_Unified(30302);
static volatile bool magDataReady = false;
static volatile bool magDataRead = false;

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(30304);
static float temperature;
static float pressure;

//#define TRACE
#define ELAPSED
//#define INTERRUPTS
#define POLLINGINTERRUPTS

#define GYRO
//#define ACCEL
//#define MAG
//#define TEMP
//#define PRESSURE

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

void setup()
{
	Serial.begin(250000);
	Serial.println("Starting up...");

#if defined GYRO
	pinMode(GRDY, INPUT);
	attachInterrupt(digitalPinToInterrupt(GRDY), GyroDataReady, RISING);
	gyro.enableAutoRange(true);
	if (!gyro.begin())
	{
		Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
	}
	//gyro.setOutputDataRate(GYRO_ODR_95);
	gyro.enableDRDYInterrupt(true);
#endif

#if defined ACCEL
	pinMode(LIN1, INPUT);
	attachInterrupt(digitalPinToInterrupt(LIN1), AccelDataReady, RISING);
	accel.enableAutoRange(true);
	if (!accel.begin())
	{
		Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
	}
	//accel.setAccelRate(LSM303_ACCEL_ODR_100);       // Default is 100 Hz
	accel.enableInt1DataReady(true);
#endif

#if defined MAG
	pinMode(LRDY, INPUT);
	attachInterrupt(digitalPinToInterrupt(LRDY), MagDataReady, RISING);      // DRDY/LRDY, Mag Data Ready
	mag.enableAutoRange(true);
	if (!mag.begin()) {
		Serial.println(F("Ooops, no LSM303 detected (mag)... Check your wiring!"));
	}
	//mag.setMagRate(LSM303_MAGRATE_75);              // Default is 15 Hz
	// mag interrupt is on by default
#endif

#if defined PRESSURE || defined TEMP
	if (!bmp.begin())
	{
		Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
	}
#endif
}

static unsigned long start;

void loop()
{
#ifdef GYRO
#if defined INTERRUPTS
	if (gyroDataReady)
#elif defined POLLINGINTERRUPTS
	if (digitalRead(GRDY))
#endif
	{
		start = micros();
		if (gyro.getGyro(&vector))
		{
#if defined ELAPSED
			Serial.print("GE: "); Serial.println(micros() - start);
#elif defined TRACE
			Serial.print("G");
#else
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
#endif

#ifdef ACCEL
#if defined INTERRUPTS
	if (accelDataReady)
#elif defined POLLINGINTERRUPTS
	if (digitalRead(LIN1))
#endif
	{
		start = micros();
		if (accel.getEvent(&vector))
		{
#if defined ELAPSED
			Serial.print("AE: "); Serial.println(micros() - start);
#elif defined TRACE
			Serial.print("A");
#else
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
#endif

#ifdef MAG
#if defined INTERRUPTS
	if (magDataReady)
#elif defined POLLINGINTERRUPTS
	if (digitalRead(LRDY))
	{
	if (!magDataRead)
#endif
	{
		start = micros();
		if (mag.getEvent(&vector))
		{
#if defined ELAPSED
			Serial.print("ME: "); Serial.println(micros() - start);
#elif defined TRACE
			Serial.print("M");
#else
			Serial.print("Mag: "); Serial.print(vector.x); Serial.print(","); Serial.print(vector.y); Serial.print(","); Serial.println(vector.z);
#endif
			magDataReady = false;
#if defined POLLINGINTERRUPTS
			magDataRead = true;
#endif
		}
		else
		{
			// Don't reset the data ready flag, since we didn't actually clear the data.
			Serial.println("### Mag Error ###");
		}
	}
#if defined POLLINGINTERRUPTS
	}
	else
	{
		magDataRead = false;
	}
#endif
#endif

	start = micros();
#if defined PRESSURE || defined TEMP
#if defined INTERRUPTS || defined POLLINGINTERRUPTS
	int8_t bmpDataReady = bmp.isDataReady();
#if defined ELAPSED
	unsigned long dr = micros() - start;
#endif
	if (bmpDataReady > 0)
#endif
	{
#if defined TEMP
#if defined INTERRUPTS || defined POLLINGINTERRUPTS
		if (bmpDataReady == DATA_READY_TEMPERATURE)
		{
#endif
		start = micros();
		if (bmp.getTemperature(&temperature))
		{
#if defined ELAPSED
			Serial.print("EDT: "); Serial.print(dr); Serial.print("; ET: "); Serial.println(micros() - start);
#elif defined TRACE
			Serial.print("T");
#else
			Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" C");
#endif
		}
		else
		{
			Serial.println("### Temp Error ###");
		}
#if defined INTERRUPTS || defined POLLINGINTERRUPTS
		}
#if defined PRESSURE
		else
#endif
#endif
#endif
#if defined PRESSURE
#if defined INTERRUPTS || defined POLLINGINTERRUPTS
		if (bmpDataReady == DATA_READY_PRESSURE)
		{
#endif
		start = micros();
		if (bmp.getPressure(&pressure))
		{
			//pressure = sensors_event.pressure;
#if defined ELAPSED
			Serial.print("EDP: "); Serial.print(dr); Serial.print("; EP: "); Serial.println(micros() - start);
#elif defined TRACE
			Serial.print("P");
#else
			Serial.print("Pressure: "); Serial.print(pressure); Serial.print(" hPa; ");
			Serial.print("Altitude: "); Serial.print(bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, pressure)); Serial.println(" m");
#endif
		}
		else
		{
			Serial.println("### Alt Error ###");
		}
#if defined INTERRUPTS || defined POLLINGINTERRUPTS
		}
#endif
#endif
	}
#if defined INTERRUPTS || defined POLLINGINTERRUPTS
	else
#endif
	if (bmpDataReady < 0)
	{
		Serial.println("### BMP Error ###");
	}
#endif
}


