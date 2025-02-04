import requests
import csv
import time
from datetime import datetime

# Replace with your ESP32's IP address
ESP32_IP = "172.20.10.10"  # Check Serial Monitor for actual IP
URL = f"http://{ESP32_IP}/data"

# CSV file to store data
CSV_FILE = "sensor_data.csv"

# Function to fetch sensor data
def fetch_sensor_data():
    try:
        response = requests.get(URL, timeout=5)
        if response.status_code == 200:
            data = response.json()  # Parse JSON response
            return data
        else:
            print(f"Error: HTTP {response.status_code}")
            return None
    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")
        return None

# Initialize CSV file with headers
with open(CSV_FILE, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Timestamp", "Solid Level (cm)", "Liquid Level (cm)", "Temperature (°C)", "Humidity (%)"])
print(f"CSV file '{CSV_FILE}' initialized.")

# Run data collection loop
if __name__ == "__main__":
    try:
        while True:
            data = fetch_sensor_data()
            if data:
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                solid_level = data["solid_level"]
                liquid_level = data["liquid_level"]
                temperature = data["temperature"]
                humidity = data["humidity"]

                # Write data to CSV
                with open(CSV_FILE, mode="a", newline="") as file:
                    writer = csv.writer(file)
                    writer.writerow([timestamp, solid_level, liquid_level, temperature, humidity])
                
                print(f"Logged: {timestamp} | Solid: {solid_level} cm | Liquid: {liquid_level} cm | Temp: {temperature}°C | Humidity: {humidity}%")
            time.sleep(2)  # Fetch data every 2 seconds
    except KeyboardInterrupt:
        print("\nData collection stopped.")
