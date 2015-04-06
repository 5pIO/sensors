Hello!

Today, I start on a new series of posts based on Sensors. This first post on the series is about the DS18B20 temperature sensor.

###Things Needed

*	DS18B20 Temperature Sensor
*	4.7K Resistor
*	MCU

I actually picked up a [board](http://www.dx.com/p/ds18b20-digital-temperature-sensor-module-for-arduino-55-125-c-135047#.VSKOuBOUeQw) that already had the resistor in it from [DX](http://dx.com).

###Introduction

The DS18B20 is also called a 1 wire digital temperature sensor. It is very convenient because it just requires 1 communication wire to communicate with an MCU.

This sensor uses Dallas Semiconductor's(now Maxim) [1-wire](http://www.maximintegrated.com/en/app-notes/index.mvp/id/3989) protocol. This 1-wire protocol allows an array of devices(from the 1-wire family) to be controlled with just 1 communication wire. You can read about it in more detail by reading the information on the link above.

###Raspberry Pi
####Setup

The Raspberry Pi OS(Raspbian) already comes baked in with an interface to communicate with the 1-wire protocol. To get it running you need to follow these steps:

*	Add `dtoverlay=w1-gpio` to the `/boot/config.txt` file on your Pi. Reboot.
*	You then need to enable the 1 wire module with `sudo modprobe w1-gpio`.
*	You also need to enable the thermometer module with `sudo modprobe w1-therm`.
*	You should now see a folder like `/sys/bus/w1/devices/28-xxxx`
*	Inside this folder you will find a file called `w1-slave` which you can `cat` to see the temperature value of the sensor

You will see something like:
<img src="/content/images/2015/04/DS18B20-tempReading.png" />

Here you can see there are 2 lines of output, each line prepended by an address(which is the address for the sensor). On the first line, you first see the CRC which is just a check for the integrity of the data, you will see that if this fails the end of the first line will have a NO, which means that the data is unreliable.

The next line actually gives us the temperature in 1/1000° C. So in this example, the temperature is actually 25.562° C.

####Wiring
<img src="/content/images/2015/04/PiDS18B20.png" />

Just make sure the direction of the sensor is correct!

####Code
As stated in the the introduction, the way to read the data is actually by reading a file, so our program will do exactly that!

```
import os
import glob
import time

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'

def read_temp_raw():
  f = open(device_file, 'r')
  lines = f.readlines()
  f.close()
  return lines

def read_temp():
  lines = read_temp_raw()
  while lines[0].strip()[-3:] != 'YES':
    time.sleep(0.2)
    lines = read_temp_raw()
  equals_pos = lines[1].find('t=')
  if equals_pos != -1:
    temp_string = lines[1][equals_pos+2:]
    temp_c = float(temp_string) / 1000.0
    return temp_c

while True:
  print(read_temp())
  time.sleep(1)
```

After running this program, you will get an output like:
<img src="/content/images/2015/04/DS18B20-python.png" />
with a new value every second.

Thats it!!

###Beaglebone Black
####Setup
Unfortunately, the default kernel for for the BBB does not come pre-compiled with support for the one wire protocol. You should follow the instructions [here](http://hipstercircuits.com/dallas-one-wire-temperature-reading-on-beaglebone-black-with-dto/) to compile the device tree.

####Wiring
<img src="/content/images/2015/04/BBB-DS18B20.png" />

####Code

The code is the same as the raspberry pi above:
```
import os
import glob
import time

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'

def read_temp_raw():
  f = open(device_file, 'r')
  lines = f.readlines()
  f.close()
  return lines

def read_temp():
  lines = read_temp_raw()
  while lines[0].strip()[-3:] != 'YES':
    time.sleep(0.2)
    lines = read_temp_raw()
  equals_pos = lines[1].find('t=')
  if equals_pos != -1:
    temp_string = lines[1][equals_pos+2:]
    temp_c = float(temp_string) / 1000.0
    return temp_c

while True:
  print(read_temp())
  time.sleep(1)
```

After running this program, you will get an output like:
<img src="/content/images/2015/04/ds18b20-bbb.png" />
with a new value every second.

###Arduino
####Setup
For the Arduino, we will be using both the [OneWire](http://playground.arduino.cc/Learning/OneWire) and [DallasTemperature](http://milesburton.com/Main_Page?title=Dallas_Temperature_Control_Library) libraries. These libraries take or interfacing with sensors.

To get started you will need to find the address of your sensor. To this I used [this](http://www.hacktronics.com/Tutorials/arduino-1-wire-address-finder.html) sketch. Which just prints out the library to the serial monitor of the arduino.

####Wiring
<img src="/content/images/2015/04/ArduinoDS18B20.png" />

####Code
Thanks to the libraries, the code is pretty simple. All we do is initialize a OneWire object and then initialize the Temperature Sensor library with it. After that we just ask for the temperature from the library and print it to the serial console. The code should look something like this:

```
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into pin 3 on the Arduino
#define ONE_WIRE_BUS 3

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Assign the addresses of your 1-Wire temp sensors.
// See the tutorial on how to obtain these addresses:
// http://www.hacktronics.com/Tutorials/arduino-1-wire-address-finder.html

DeviceAddress thermometer = { 0x28, 0xFF, 0x80, 0x15, 0x54, 0x14, 0x01, 0x56 };

void setup(void)
{
  // start serial port
  Serial.begin(9600);
  // Start up the library
  sensors.begin();
  // set the resolution to 10 bit (good enough?)
  sensors.setResolution(thermometer, 10);
}

void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == -127.00) {
    Serial.print("Error getting temperature");
  } else {
    Serial.print("C: ");
    Serial.print(tempC);
  }
}

void loop(void)
{
  delay(2000);
  Serial.print("\nGetting temperature...\n\r");
  sensors.requestTemperatures();

  Serial.print("Temperature is: ");
  printTemperature(thermometer);
}
```

And thats it, just open up your serial monitor and you should see something like this:
<img src="/content/images/2015/04/Arduino-DSB1820.png" />


###Repository

You can find the code in the [Sensors](https://github.com/5pIO/sensors) repository [here](https://github.com/5pIO/sensors/tree/master/Temperature/DS18B20).
