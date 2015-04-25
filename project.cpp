
#include "grove.h"
#include "jhd1313m1.h"
#include "grovemoisture.h"

#include <climits>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include "ublox6.h"
#include "mma7455.h"
//#include "TinyGPS++.h"

using namespace upm;
using namespace std;

/*
 * Grove Starter Kit example
 *
 * Demonstrate the usage of various component types using the UPM library.
 *
 * - digital in: GroveButton connected to the Grove Base Shield Port D4
 * - digital out: GroveLed connected to the Grove Base Shield Port D3
 * - analog in: GroveTemp connected to the Grove Base Shield Port A0
 * - I2C: Jhd1313m1 LCD connected to any I2C on the Grove Base Shield
 *
 * Additional linker flags: -lupm-i2clcd -lupm-grove
 */

/*
 * Update the temperature values and reflect the changes on the LCD
 * - change LCD backlight color based on the measured temperature,
 *   a cooler color for low temperatures, a warmer one for high temperatures
 * - display current temperature
 * - record and display MIN and MAX temperatures
 * - reset MIN and MAX values if the button is being pushed
 * - blink the led to show the temperature was measured and data updated
 */
void temperature_update(upm::GroveTemp* temperature_sensor, upm::GroveButton* button,
		upm::GroveLed* led, upm::Jhd1313m1 *lcd, upm::GroveRotary* rotary)
{
	// minimum and maximum temperatures registered, the initial values will be
	// replaced after the first read
	static int min_temperature = INT_MAX;
	static int max_temperature = INT_MIN;

	// the temperature range in degrees Celsius,
	// adapt to your room temperature for a nicer effect!
	const int TEMPERATURE_RANGE_MIN_VAL = 18;
	const int TEMPERATURE_RANGE_MAX_VAL = 31;

	// other helper variables
	int temperature; // temperature sensor value in degrees Celsius
	float fade; // fade value [0.0 .. 1.0]
	uint8_t r, g, b; // resulting LCD backlight color components [0 .. 255]
	std::stringstream row_1, row_2; // LCD rows

	// update the min and max temperature values, reset them if the button is
	// being pushed
	temperature = temperature_sensor->value();
	if (button->value() == 1) {
		min_temperature = temperature;
		max_temperature = temperature;
	} else {
		if (temperature < min_temperature) {
			min_temperature = temperature;
		}
		if (temperature > max_temperature) {
			max_temperature = temperature;
		}
	}
	float rotary_val = rotary->abs_deg();
	// display the temperature values on the LCD
	row_1 << "Rot " << rotary_val << "    ";
	row_2 << "Temp " << temperature << "    ";
	lcd->setCursor(0,0);
	lcd->write(row_1.str());
	lcd->setCursor(1,0);
	lcd->write(row_2.str());

	// set the fade value depending on where we are in the temperature range
	if (temperature <= TEMPERATURE_RANGE_MIN_VAL) {
		fade = 0.0;
	} else if (temperature >= TEMPERATURE_RANGE_MAX_VAL) {
		fade = 1.0;
	} else {
		fade = (float)(temperature - TEMPERATURE_RANGE_MIN_VAL) /
				(TEMPERATURE_RANGE_MAX_VAL - TEMPERATURE_RANGE_MIN_VAL);
	}

	// fade the color components separately
	r = (int)(255 * fade);
	g = (int)(64 * fade);
	b = (int)(255 * (1 - fade));

	// blink the led for 50 ms to show the temperature was actually sampled
	led->on();
	usleep(50000);
	led->off();

	// apply the calculated result
	lcd->setColor(r, g, b);
}

string parseGPGGA( char* indata){
	string stringres;
	char* dolar = strchr(indata,'$');
	if(dolar!=NULL)
	{
		string indata_s = string(indata);
		if(dolar-indata==0)
		{
			string strgpgga = indata_s.substr (1,5);
			if(strgpgga.compare("GPGGA")==0)
			{
				return indata_s.substr(18,25);
//				int koma = 0;
//				cout<< "hasil "<<outdata;
//				cout << '\n';
			}
			else
			{
				return "data invalid\n";
			}
		}
	}
}

string gps_instance(upm::Ublox6* nmea)
{
	 cout << "masuk "<<endl;
	 char nmeaBuffer[70];

	 bool checkGPS = true;
	 while(checkGPS)
	 {
	      // we don't want the read to block in this example, so always
	      // check to see if data is available first.
	      if (nmea->dataAvailable())
	        {
	          int rv = nmea->readData(nmeaBuffer, 70);
	          if (rv > 0)
	          {
	        	  write(1, nmeaBuffer, rv);
	          }

	          if (rv < 0) // some sort of read error occured
	            {
	              cerr << "Port read error." << endl;
	              break;
	            }
	          continue;
	        }
//	      cout<<"AAAAAA"<<endl;
	      usleep(100000); // 100ms


		if(strlen(nmeaBuffer)>69)
		{
			checkGPS=false;

		}
	 }


	  cout<<"nmea : "<<nmeaBuffer<<endl;

	  return parseGPGGA(nmeaBuffer);

}

int main()
{
	// check that we are running on Galileo or Edison
	mraa_platform_t platform = mraa_get_platform_type();
	if ((platform != MRAA_INTEL_GALILEO_GEN1) &&
			(platform != MRAA_INTEL_GALILEO_GEN2) &&
			(platform != MRAA_INTEL_EDISON_FAB_C)) {
		std::cerr << "Unsupported platform, exiting" << std::endl;
		return MRAA_ERROR_INVALID_PLATFORM;
	}

	upm::GroveRotary* rotary = new upm::GroveRotary(1);

	// button connected to D4 (digital in)
	upm::GroveButton* button = new upm::GroveButton(4);

	// led connected to D3 (digital out)
	upm::GroveLed* led = new upm::GroveLed(3);

	// temperature sensor connected to A0 (analog in)
	upm::GroveTemp* temp_sensor = new upm::GroveTemp(0);

	// LCD connected to the default I2C bus
	upm::Jhd1313m1* lcd = new upm::Jhd1313m1(0);

	// GPS UART
	upm::Ublox6* nmea = new upm::Ublox6(0);
	int bufferLength = 70;
	string coordinate;


	//Light
	upm::GroveLight* light = new upm::GroveLight(2);

	//Accelerometer
    int16_t *raw;
    float *acc;
    // Note: Sensor only works at 3.3V on the Intel Edison with Arduino breakout
    upm::MMA7455 *accel = new upm::MMA7455(0, ADDR);

    //Moisture
    upm::GroveMoisture* moisture = new GroveMoisture(3);
	// simple error checking
	if ((button == NULL) || (led == NULL) || (temp_sensor == NULL) || (lcd == NULL)) {
		std::cerr << "Can't create all objects, exiting" << std::endl;
		return MRAA_ERROR_UNSPECIFIED;
	}

	// loop forever updating the temperature values every second
	for (;;) {
		temperature_update(temp_sensor, button, led, lcd, rotary);
		 short x, y, z;
		accel->readData(&x, &y, &z);
		std::cout << "Accelerometer X(" << x << ") Y(" << y << ") Z(" << z << ")" << std::endl;
		        usleep (100000);

		        //Light
//		        cout << light->name() << " raw value is " << light->raw_value() << ", which is roughly " << light->value() << " lux" << endl;

		        //Moisture
//		        int val = moisture->value();
//		              cout << "Moisture value: " << val << ", ";
//		              if (val >= 0 && val < 300)
//		                cout << "dry";
//		              else if (val >= 300 && val < 600)
//		                cout << "moist";
//		              else
//		                cout << "wet";
//		              cout << endl;

		        //GPS

		    	// make sure port is initialized properly.  9600 baud is the default.
		    	  if (!nmea->setupTty(B9600))
		    	  {
		    		  cerr << "Failed to setup tty port parameters" << endl;
		    	  }
		    	  else
		    	  {
//		    		  coordinate = gps_instance( nmea);
//		    		  memset(nmeaBuffer, 0, bufferLength);
		    	  }
		sleep(1);
	}

	return MRAA_SUCCESS;
}
