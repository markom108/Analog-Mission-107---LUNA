# Autonomous Plant Growth & Environmental Monitoring Station 🌿📸

This repository contains the software for an autonomous, energy-efficient monitoring station designed for isolated environments, such as analog space habitats. The system continuously tracks the visual growth of plants alongside surrounding microclimatic conditions.

It was initially deployed and tested during **Analog Mission 107 (LUNA)** at the Analog Astronaut Training Center.

## 🚀 Project Overview

The station captures environmental data (temperature, humidity, atmospheric pressure, and light intensity) and pairs it with high-resolution images at regular, configurable intervals (10 minutes by default). The data is stored locally on an SD card, creating a synchronized dataset suitable for analyzing plant growth dynamics and health without requiring external network infrastructure.

## 🛠️ Hardware Components

* **Microcontroller:** ESP32-CAM (featuring an integrated OV2640 camera module)
* **Sensors:**
  * `BME280` (Temperature, Relative Humidity, Atmospheric Pressure)
  * `BH1750` (Light Intensity)
* **Storage:** MicroSD Card (16-32 GB) for local data logging
* **Power:** 4x AA batteries (6.0V nominal), regulated down through the ESP32-CAM-MB's onboard 5V input

## ⚡ Key Features & Software Architecture

* **Extreme Energy Efficiency:** The standard microcontroller execution paradigm was fundamentally altered. All operations (sensor readings, camera capture, SD card writes) are executed strictly within `setup()`. The system then immediately enters **Deep Sleep**, bypassing the power-draining `loop()` entirely.
* **100% Offline Autonomy:** Wi-Fi and NTP synchronization are deliberately disabled to save energy and eliminate dependency on unstable external networks. Timestamps are calculated from a configured start date/time combined with the current series number — no network time source is needed.
* **Synchronized Logging:** Sensor data is written with semicolon-delimited fields and appended to a `data.csv` file, with each row linked to its corresponding image filename.

## ⚙️ Pin Configuration (I2C)

Due to the limited GPIO availability on the ESP32-CAM, the UART-designated pins were repurposed for the I2C interface:

* **SDA:** GPIO1 (U0TXD)
* **SCL:** GPIO3 (U0RXD)

## 🔧 Configuration

Key parameters can be adjusted at the top of the source file before flashing:

* `START_SERIES` — first series number for this deployment (check the SD card for the last saved series if resuming)
* `START_YEAR` / `START_MONTH` / `START_DAY` / `START_HOUR` / `START_MINUTE` / `START_SECOND` — reference start date/time used to calculate all timestamps
* `INTERVAL_MIN` — minutes between photo series (default: 10)
* `PHOTO_COUNT` — number of photos per series (default: 3)
* `PHOTO_DELAY_SEC` — seconds between photos within a series (default: 5)
* `CAMERA_RESOLUTION` / `JPEG_QUALITY` — image resolution and compression

## 🚀 Usage

1. Upload the code to the ESP32-CAM using an ESP32-CAM-MB USB programmer (remove the SD card during upload to avoid conflicts).
2. Insert a formatted MicroSD card.
3. Wire the sensors and connect the battery pack to the board's 5V input.
4. The system will automatically wake up, take readings/photos, log the data, and return to deep sleep, repeating at the configured interval.
