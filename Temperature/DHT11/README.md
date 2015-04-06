Hello!!

Continuing with our [Sensor Series](http://5p.io/tag/sensor-series/), we will work with a different temperature sensor today. This sensor is the DHT11, which is a temperature and humidity sensor.

###Things Needed
*	DHT11(/22) Sensor
*	4.7K Resistor
*	MCU(Pi/BBB/Arduino)

###Introduction
The DHT11 sensor consists of a capacitive humidity sensor and a thermistor to measure the temperature of the air around it. It is an ultra low cost digital sensor with a
calibrated digital signal output. It is great for all sorts of DIY projects, however it does have 1 drawback. It can only check for temperature and humidity once every 2 seconds, which means the readings can be up to 2 seconds old. For more information you should look at the [datasheet](http://www.micropik.com/PDF/dht11.pdf)

###Raspberry Pi
####Setup
To interface with this sensor we will be using a [library](https://github.com/adafruit/Adafruit_Python_DHT) from the good folks over at [Adafruit](http://adafuit.com). You can install the library with the following steps:

*	`git clone https://github.com/adafruit/Adafruit_Python_DHT`
*	`cd Adafruit\_Python\_DHT && sudo python setup.py install`

####Wiring
<img src="/content/images/2015/04/piDHT11-1.png" />

Make sure you have connected a 4.7K pull up resistor  to the data pin.

####Code
Here, the library does most of the work for us, all we need to do is initialize it with the correct pin and ask it to read the data.

```
import Adafruit_DHT

sensor = Adafruit_DHT.DHT11

sensor_pin = 4

hum, temp = Adafruit_DHT.read_retry(sensor, sensor_pin)

if hum is not None and temp is not None:
  print 'Temp={0:0.1f}*C  Humidity={1:0.1f}%'.format(temp, hum)
```

Thats it!! Just run the program and you should get the current temperature and humidity readings.


###Beaglebone Black
####Setup
We will be using the same library for the BBB as well, you can set it up by following the same instructions:

*	`git clone https://github.com/adafruit/Adafruit_Python_DHT`
*	`cd Adafruit\_Python\_DHT && sudo python setup.py install`

####Wiring
<img src="/content/images/2015/04/BBBDHT11.png" />

Once again, don't forget the 4.7K pullup resistor for the data pin.

####Code
The code here is exactly the same as the Pi, except the pin numbers.

```
import Adafruit_DHT

sensor = Adafruit_DHT.DHT11

sensor_pin = 'P8_11'

hum, temp = Adafruit_DHT.read_retry(sensor, sensor_pin)

if hum is not None and temp is not None:
  print 'Temp={0:0.1f}*C  Humidity={1:0.1f}%'.format(temp, hum)
```
And once agin, just run the program and you should be able to see all the readings.


###Arduino

####Setup
For the Arduino, we will once again be using a [library](https://github.com/adafruit/DHT-sensor-library) from Adafruit. You can install it by downloading the zip file from Github and importing it into the Arduino IDE.

####Wiring
<img src="/content/images/2015/04/ArduinoDHT11.png" />

Once again, don't forget the 4.7K pullup resistor for the data pin.

####Code
As the library does most of the heavy lifting, All we do is initialize the `dht` object with the pin number and type, and then ask the library to read the sensor data. The code should look something like this:

```
#include <DHT.h>

#define DHTPIN 2

#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);


void setup() {
  Serial.begin(9600);
  dht.begin();
}

void loop() {
  // Wait a few seconds between measurements.
  delay(2000);

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C \n");
}
```

And *voila*!! You are done!! Just open the Serial monitor and you should be seeing the readings.

###Repository

You can find all the code used here in the [Sensors](http://github.com/5pIO/sensors) repository [here](https://github.com/5pIO/sensors/tree/master/Temperature/DHT11).



