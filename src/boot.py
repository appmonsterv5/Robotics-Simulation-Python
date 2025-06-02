# boot.py (for ESP32)
# Save this file as boot.py on your ESP32 device.
# It will run automatically when the ESP32 boots.

import network
import time
import machine
import main

# --- Wi-Fi Credentials ---
WIFI_SSID = "ABC"  # Replace with your Wi-Fi network name
WIFI_PASSWORD = "123456789"  # Replace with your Wi-Fi password

def connect_wifi(max_retries=10):
    """Connects the ESP32 to Wi-Fi."""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(False)  # Disable Wi-Fi to reset the state
    time.sleep(1)       # Wait briefly
    wlan.active(True)   # Enable Wi-Fi again

    print("Connecting to WiFi...", end="")
    retries = 0

    # Try connecting with a timeout for each attempt
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    
    while not wlan.isconnected() and retries < max_retries:
        print(".", end="")
        time.sleep(1) # Wait 1 second before checking again
        retries += 1
        if retries % 30 == 0: # Try to reconnect every 30s if first connect fails
            print("Re-attempting connect...")
            wlan.connect(WIFI_SSID, WIFI_PASSWORD)


    if wlan.isconnected():
        print("\nWiFi connected successfully!")
        print("Network configuration:", wlan.ifconfig())
    else:
        print("\nFailed to connect to WiFi after multiple retries.")
        print("Please check your credentials and network.")
        print("Restarting device to try again...")
        time.sleep(5)
        machine.reset() # Reset the device if connection fails

# Automatically connect to Wi-Fi on boot
if __name__ == "__main__":
    connect_wifi()
