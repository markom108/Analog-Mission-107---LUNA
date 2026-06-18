# Analog-Mission-107---LUNA
# Autonomous Plant Growth & Environmental Monitoring Station 🌿📸

This repository contains the software for an autonomous, energy-efficient monitoring station designed for isolated environments, such as analog space habitats. The system continuously tracks the visual growth of plants alongside surrounding microclimatic conditions.

It was initially deployed and tested during **Analog Mission 107 (LUNA)** at the Analog Astronaut Training Center.

## 🚀 Project Overview
The station captures vital environmental data (temperature, humidity, atmospheric pressure, and light intensity) and pairs it with high-resolution images at precise 10-minute intervals. The data is stored locally, creating a synchronized dataset perfect for analyzing plant growth dynamics and health without requiring external network infrastructure.

## 🛠️ Hardware Components
* **Microcontroller:** ESP32-CAM (featuring an integrated OV2640 camera module)
* **Sensors:** * `BME280` (Temperature, Relative Humidity, Atmospheric Pressure)
    * `BH1750` (Light Intensity)
* **Storage:** MicroSD Card (16-32 GB) for local data logging
* **Power:** 4x AA batteries (providing a stable 6.0V nominal input to prevent brownout resets)

## ⚡ Key Features & Software Architecture
* **Extreme Energy Efficiency:** The standard microcontroller execution paradigm was fundamentally altered. All operations (sensor readings, camera capture, SD card writes) are executed strictly within the `setup()` function. The system then immediately enters **Deep Sleep**, bypassing the power-draining `loop()` entirely.
* **100% Offline Autonomy:** Wi-Fi and NTP synchronization are deliberately disabled to save massive amounts of energy and eliminate dependency on unstable external networks. Timestamps are derived mathematically from session counters.
* **Synchronized Logging:** Data is formatted with semicolon delimiters and appended to a `data.csv` file alongside the corresponding image filenames.

## ⚙️ Pin Configuration (I2C)
Due to the limited GPIO availability on the ESP32-CAM, UART-designated pins were repurposed for the I2C interface:
* **SDA:** GPIO1 (U0T)
* **SCL:** GPIO3 (U0R)

## 🚀 Usage
1. Upload the code to the ESP32-CAM using an ESP32-CAM-MB USB programmer (ensure the SD card is removed during upload to avoid conflicts).
2. Insert a formatted MicroSD card.
3. Wire the sensors and connect the battery pack to the 5V input.
4. The system will automatically wake up, take readings/photos, log the data, and return to sleep every 10 minutes.
