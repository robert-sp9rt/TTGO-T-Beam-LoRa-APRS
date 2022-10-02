<h2>Installation Guide using PlatformIO</h2>
<br>
1. Clone all files of the respository to your local working directory<br>
2. Install the missing libraries<br>
There are two possibilities - either using the Library Manager of PlatformIO or the command line tool:<br>
<h3>1. Built-In Liabrary Manager</h3>
Press the PlatformIO HOME Button to enter the Home Screen and there the Libraries Button to add missing libraries.<br>
Search and install the following libaries:<br/>
<ul>
<li>RadioHead</li>
<li>TinyGPSPlus</li>
<li>DHT sensor library for ESPx</li>
<li>Adafruit SSD1306</li>
<li>Adafruit GFX Library</li>
<li>Adafruit BusIO</li>
<li>Adafruit Unified Sensor</li>
<li>https://github.com/SQ9MDD/AXP202X_Library.git (AXP202X_Library)</li>
<li>OneWire</li>
<li>DallasTemperature</li>
<li>SparkFun u-blox Arduino Library</li>
<li>blanchon/ArduinoJson</li>
<li>arcao/Syslog</li>
</ul>
<br>
<h3>2. Command Line Tool</h3>
If you issue<br/>
~/.platformio/penv/bin/pio run<br/>
It should resolve all dependencies automaticaly, thanks to the definitions in the section "lib_deps =".<br/>
You could (but not need to) install the libraries by hand:
<br/>
platformio lib install "&lt;library name&gt;“
<br/>
Check that the platformio.ini is available as it holds the board type for PlatformIO.

<h2>Compile</h2>
<ul>
<li>GUI: After pressing the check mark the code will be compiled, after pressing the arrow it will be compiled and uploaded to a connected TTGO.</li>
<li>platformio-cli: ~/.platformio/penv/bin/pio run -e ttgo-t-beam-v1.0 -t upload --upload-port /dev/cu.SLAB_USBtoUART</li>
</ul>
