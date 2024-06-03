import time
from time import localtime, strftime
import board
import adafruit_dht
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from digitalio import DigitalInOut, Direction
import atexit
import math
import paho.mqtt.client as mqtt

# Your MQTT-Broker-Configuration
MQTT_BROKER_HOST = "***.YOUR.IP.***"
MQTT_BROKER_PORT = 1883
MQTT_TOPIC = "YOUR MQTT TOPIC"

# MQTT-Client-Initialisation
mqtt_client = mqtt.Client()

# Thresholds and hysteresis
SWITCH_min = 2.25  # Minimum dew point difference for the relay to switch
HYSTERESIS = 0.5   # Hysteresis value for switching
TEMP1_min = 2      # Minimum indoor temperature
TEMP2_min = -10    # Minimum outdoor temperature

# Sensor correction values
Correction_t_1 = 0     # Correction value for indoor temperature sensor
Correction_t_2 = 0.1   # Correction value for outdoor temperature sensor
Correction_h_1 = 0     # Correction value for indoor humidity sensor
Correction_h_2 = -3.3  # Correction value for outdoor humidity sensor

# Set update rate for sensor reading (in seconds)
UPDATETIME = 30
start = time.time()

# Define GPIO for relay board
relay_1 = DigitalInOut(board.D21)
relay_1.direction = Direction.OUTPUT
relay_1.value = True

# Initialize the dht device, with data pin connected to:
dhtDevice1 = adafruit_dht.DHT22(board.D16) # Indoor Sensor
dhtDevice2 = adafruit_dht.DHT22(board.D20) # Outdoor Sensor

# Set LCD size
lcd_columns = 16
lcd_rows = 2
# Initialize I2C bus
i2c = board.I2C()  # uses board.SCL and board.SDA
# Initialize the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()
# Set LCD color to blue
lcd.color = [100, 0, 0]
# Print two-line message
lcd.message = "Prototype v0.7\nDewPointFanCtl"
time.sleep(2)
lcd.clear()

# Define global variables
temperature_c_1 = temperature_c_2 = humidity_1 = humidity_2 = dewpoint_1 = dewpoint_2 = menu = lastmenu = deltaDP = absoluteHumidity_1 = absoluteHumidity_2 = 0
rel = False

# Function to read DHT sensors
def read_dht():
	try:
		global temperature_c_1, temperature_c_2, humidity_1, humidity_2, dewpoint_1, dewpoint_2, deltaDP, rel
		temperature_c_1 = round(dhtDevice1.temperature + Correction_t_1,2)
		humidity_1 = round(dhtDevice1.humidity + Correction_h_1,2)
		temperature_c_2 = round(dhtDevice2.temperature + Correction_t_2,2)
		humidity_2 = round(dhtDevice2.humidity + Correction_h_2,2)
		dewpoint_1 = round(calc_dewpoint(temperature_c_1,humidity_1),2)
		dewpoint_2 = round(calc_dewpoint(temperature_c_2,humidity_2),2)
		deltaDP = round(dewpoint_1 - dewpoint_2,2)
		absoluteHumidity_1 = round(calculate_absolute_humidity(temperature_c_1,humidity_1),2)
		absoluteHumidity_2 = round(calculate_absolute_humidity(temperature_c_2,humidity_2),2)
		if deltaDP == None:
			deltaDP = 0
		
		print("Indoor:  {:.1f} C    {:.1f} %    {:.1f} C   {:.1f} g/m^3".format(temperature_c_1, humidity_1, dewpoint_1, absoluteHumidity_1))
		print("Outdoor: {:.1f} C    {:.1f} %    {:.1f} C   {:.1f} g/m^3".format(temperature_c_2, humidity_2, dewpoint_2, absoluteHumidity_2))
		print("DELTA:   {:.1f} C    {:.1f} %    {:.1f} C   {:.1f} g/m^3".format(temperature_c_1-temperature_c_2, humidity_1-humidity_2, dewpoint_1-dewpoint_2, absoluteHumidity_1-absoluteHumidity_2))

		relay(dewpoint_1,dewpoint_2)

		lcd.home()
		lcd.message = ("{:.1f}C {:.0f}% {:.1f}C".format(temperature_c_1, humidity_1, dewpoint_1))
		lcd.cursor_position(0,1)
		lcd.message = ("{:.1f}C {:.0f}% {:.1f}C".format(temperature_c_2, humidity_2, dewpoint_2))

		send_mqtt_data(temperature_c_1, humidity_1, absoluteHumidity_1, temperature_c_2, humidity_2, absoluteHumidity_2, rel)

	except RuntimeError as error:
		# Errors happen fairly often, DHT's are hard to read, just keep going
		print(error.args[0])
		time.sleep(2.0)
	except Exception as error:
		dhtDevice1.exit()
		dhtDevice2.exit()
		raise error
	else:
	    pass

# Function to control relay based on conditions
def relay(t1,t2):
    global deltaDP
    global rel

    if deltaDP > (SWITCH_min + HYSTERESIS):
        rel = True
    if deltaDP < SWITCH_min:
        rel = False
    if t1 < TEMP1_min:
        rel = False
    if t2 < TEMP2_min:
        rel = False

    if rel == True:
        relay_1.value = False
        print("FAN on")
    else:
        relay_1.value = True
        print("FAN off")

# Function to handle button inputs (currently not used)
def buttons():
    global menu
    if lcd.left_button:
        lcd.message = "Left!"
    elif lcd.up_button:
        menu-=1
    elif lcd.down_button:
        menu+=1
    elif lcd.right_button:
        lcd.message = "Right!"
    elif lcd.select_button:
        lcd.message = "Select!"

# Function to display information on the LCD (currently not used)
def display():
    global temperature_c_1
    global temperature_c_2
    global humidity_1
    global humidity_2
    global menu
    global lastmenu

    if menu > 2 or menu < 0:
        menu = 0
    if lastmenu != menu:
        lcd.clear()
        lastmnenu = menu
    elif lastmenu == menu:
        return 0

    if menu == 0:
        lcd.home()
        lcd.message = ("Status:")
        lcd.cursor_position(0,1)
        lcd.message = (f"Fan: {rel}")
    elif menu == 1:
        lcd.home()
        lcd.message = ("In")
        lcd.cursor_position(0,1)
        lcd.message = (f"{temperature_c_1}C {humidity_1}% {calc_dewpoint(temperature_c_1,humidity_1)}")
    elif menu == 2:
        lcd.home()
        lcd.message = ("Out")
        lcd.cursor_position(0,1)
        lcd.message = (f"{temperature_c_2}C {humidity_2}% {calc_dewpoint(temperature_c_2,humidity_2)}")
    lastmenu = menu

# Function to handle exit and cleanup
def exit_handler():
    print("Exiting...")
    dhtDevice1.exit()
    dhtDevice2.exit()
    relay_1.value = True
    lcd.clear()
    lcd.color = [0,0,0]
    mqtt_client.disconnect()


def calc_dewpoint(t,h):
    if t >= 0:
        A = 7.5
        B = 237.3
    elif t < 0:
        A = 7.6
        B = 240.7
    try:
        # Quelle der Formel: https://www.wetterochs.de/wetter/feuchte.html
        # Sättigungsdampfdruck in hPa
        ssd = 6.1078 * pow(10, ((A * t) / (B + t)))
        # Dampfdruck in hPa
        dd = ssd * (h / 100)
        # v-Parameter
        v = math.log((dd / 6.1078),10)
        # Taupunkttemperatur
        tt = (B * v) / (A - v)
        return tt
    except:
        return 0

def calculate_absolute_humidity(temperature_C, relative_humidity):
    if temperature_C < -40 or temperature_C > 50:
        raise ValueError("Temperature must be between -40°C and 50°C")
    if relative_humidity < 0 or relative_humidity > 100:
        raise ValueError("Relative humidity must be between 0% and 100%")

    A = (6.112 * math.exp((17.67 * temperature_C) / (temperature_C + 243.5)) * relative_humidity * 2.1674) / (273.15 + temperature_C)

    return A

def send_mqtt_data(temperature_c_1, humidity_1, absoluteHumidity_1, temperature_c_2, humidity_2, absoluteHumidity_2, rel):
    try:
        mqtt_client.username_pw_set(username="YOUR MQTT USERNAME", password="YOUR MQTT PASSWORD")
        mqtt_client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, 60)
        mqtt_client.publish(MQTT_TOPIC + "/temperatureIndoor", temperature_c_1)
        mqtt_client.publish(MQTT_TOPIC + "/humidityIndoor", humidity_1)
        mqtt_client.publish(MQTT_TOPIC + "/absoluteHumidityIndoor", absoluteHumidity_1)
        mqtt_client.publish(MQTT_TOPIC + "/temperatureOutdoor", temperature_c_2)
        mqtt_client.publish(MQTT_TOPIC + "/humidityOutdoor", humidity_2)
        mqtt_client.publish(MQTT_TOPIC + "/absoluteHumidityOutdoor", absoluteHumidity_2)
        mqtt_client.publish(MQTT_TOPIC + "/fan",rel )
        mqtt_client.disconnect()
    except Exception as e:
        print("Error sending MQTT-data:", str(e))


# Register the exit handler function
atexit.register(exit_handler)

# Main loop
while True:
    if time.time() - start > UPDATETIME:
        start = time.time()
        read_dht()
    else:
        pass
#    buttons()
#    display()
