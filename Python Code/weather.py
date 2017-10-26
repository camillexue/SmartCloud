from weather import Weather
weather = Weather()

# Lookup WOEID via http://weather.yahoo.com.

lookup = weather.lookup(560743)
condition = lookup.condition()
print(condition['text'])

# Lookup via location name.

location = weather.lookup_by_location('dublin')
condition = location.condition()
print(condition['text'])