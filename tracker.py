from skyfield.api import load, Topos, Star
from datetime import datetime
import geocoder
import os
import time
import sys
import serial
import logging

today = datetime.now().strftime("%Y%m%d")
log_filename = f"/home/ashleyjiang/startracker/logs/logfile_{today}.log"

logging.basicConfig(
    level=logging.INFO,                      # Could also be DEBUG, ERROR, WARNING, etc.
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_filename),
        logging.StreamHandler()              # Also print to console
    ]
)

logging.info('starting')

"""
Frequently used command on Rasperry Pi

pwd
ls -l

cd startracker
source star-env/bin/activate

fuser /dev/ttyACM0

sudo fuser -k /dev/ttyACM0

ps -ef |grep -i ttyACM

# for shutting down Pi
sudo shutdown -h now
sudo init 0

# for restarting Pi
sudo reboot
sudo init 6

"""

# === Serial setup to talk to Arduino
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

response = 'ACK'

#if len(sys.argv) < 2:
    #print("No target specified.")
    #sys.exit(1)
    
#target_name = sys.argv[1].strip().capitalize()


#print(f"Tracking target: {target_name}")

#if target_name == 'Jupiter':
#    target_name = target_name + ' Barycenter'

target_name = 'Jupiter Barycenter'
planets = load('de421.bsp')

logging.info('target name' + target_name)

name_to_id = {
    'Mercury': 1,
    'Venus': 2,
    'Earth': 3,
    'Mars': 4,
    'Jupiter Barycenter': 5,
    'Saturn': 6,
    'Uranus': 7,
    'Neptune': 8,
    'Pluto': 9,
    'Sun': 10,
    'Moon': 301,
}

# Get the name from the command line argument
# target_name = sys.argv[1]

#if target_name not in name_to_id:
#   raise ValueError(f"Invalid target: {target_name}")

earth = planets['earth']
target = planets[target_name]


#name = target.target_name.split(' ', 1)[1].capitalize()

#print("Tracking target until it sets...\n")

# g = geocoder.ip('me')  # Get location from public IP
# lat, lon = g.latlng    # Calculates location based on IP address
lat, lon = 39.9, 116.4    # Test coordinates

try:
    # Create observer based on IP-derived coordinates
    observer = earth + Topos(latitude_degrees = lat, longitude_degrees = lon, elevation_m = 1)
    #observer = earth + Topos(latitude_degrees=lat, longitude_degrees=lon)
    observerStatus = True    
except Exception as e:
    logging.error(f"Skyfield calc failed: {e}")

while observerStatus == True:
    ts = load.timescale()
    t = ts.now()

    try:
        # Compute Target position based on observer's position and time
        astrometric = observer.at(t).observe(target)
        alt, az, distance = astrometric.apparent().altaz()
    except Exception as e:
        logging.info(e)      
    
    #os.system('clear')
    #print(f"Observer \n  Latitude: {lat}° \n  Longitude: {lon}°\n")
    #print(f"{target_name} \n  Azimuth: {az.degrees:.2f}° \n  Altitude: {alt.degrees:.2f}° \n  Distance: {distance.km:.2f} km\n")
    
    #if alt.degrees > 0 or alt.degrees < 0:
    if alt.degrees > 0:
        command = f"{az.degrees:.2f} {alt.degrees:.2f}\n"
        logging.info(f'tx--> Target Angle: {alt.degrees:.2f} | Heading: {az.degrees:.2f}')
        
        try:
            if response == 'ACK':
                ser.write(command.encode('utf-8'))
                ser.flush()
                response = ''
                
        except Exception as e:
            logging.error(f"Serial write error: {e}")
            continue   


        try:
            if ser.in_waiting:
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                if response == "ACK":
                    logging.info("<-- rx ... Arduino ready")
                elif response == "TA":
                    logging.info(f"<--rx ...target aligned")
                    response = "ACK"
                elif response == "SR":
                    logging.info(f"<--rx !!! STOP received")
                else: 
                    logging.info(f"<--rx {response}")
                    
        except Exception as e:
            logging.error(f"Serial read error: {e}")

    else:
        logging.info("Target is below the horizon")

    time.sleep(3)  # Avoid flooding Arduino
        
