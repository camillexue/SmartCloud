import pyowm
import serial

owm = pyowm.OWM('bcd436083a8b428965a7a71697cf6c05')  # You MUST provide a valid API key

# # Will it be sunny tomorrow at this time in Milan (Italy) ?
obs = owm.weather_at_place('Vancover')
weatherCode = obs.get_weather().get_weather_code()

weatherCondition = 0
motorCounter = 0
if 200 <= weatherCode < 300:
    weatherCondition = 4
elif 300 <= weatherCode < 400:
    weatherCondition = 4
elif 400 <= weatherCode < 500:
    weatherCondition = 4
elif 500 <= weatherCode < 600:
    weatherCondition = 4
elif 600 <= weatherCode < 700:
    weatherCondition = 4
elif 700 <= weatherCode < 800:
    weatherCondition = 1
elif weatherCode == 800:
    weatherCondition = 1
elif 801 <= weatherCode < 803:
    weatherCondition = 2
elif 803 <= weatherCode < 804:
    weatherCondition = 3

print(weatherCondition)
ser = serial.Serial('/dev/ttyACM0', 9600)
while True:
   ser.write(bytes([weatherCondition]))
