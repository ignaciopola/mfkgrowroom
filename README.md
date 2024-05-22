# Grow Room Controller

## Overview
This repository contains the code and setup instructions for an Arduino-based complete automation system for managing a grow room. The system includes automated climate control, Wi-Fi and infrared communication with devices, MQTT protocol for IoT, and a user-friendly dashboard via Node-RED. It is deployed on AWS EC2 for remote access, allowing real-time monitoring and control of environmental parameters to ensure optimal growing conditions.

## Features
- **Automated Climate Control**: Manage and regulate temperature, humidity, and other environmental factors automatically.
- **Wi-Fi and Infrared Communication**: Enable wireless communication with various devices using Wi-Fi and infrared technologies.
- **MQTT Communication for IoT**: Utilize the MQTT protocol for efficient and reliable communication between IoT devices.
- **Parameter Control via Node-RED Dashboard**: Monitor and adjust system parameters through a user-friendly dashboard built in Node-RED.
- **Real-Time Data Monitoring**: Continuously track environmental conditions and system performance in real time.
- **Remote Access**: Access and control the system remotely via the AWS EC2 deployment.
- **Customizable Alerts**: Set up and receive alerts for specific conditions or thresholds, ensuring timely responses to changes.
- **Historical Data Logging**: Store and review historical data for analysis and optimization of the cultivation environment.

## Technologies
- **Programming Language**: C++ (for Arduino)
- **Database**: MySQL
- **Dashboard**: Node-RED
- **Cloud Services**: AWS EC2

## Hardware
- **Arduino Mega**
- **ESP12F NodeMCU D1 Mini ESP8266**
- **DHT22 Sensors**
- **MHZ19B CO2 Sensor**
- **Display OLED 1.3" 128x64 I2C SH1106**
- **8-Module Relay, RTC, etc.**

## Usage
- **Real-Time Monitoring**: Access the Node-RED dashboard to monitor environmental conditions and system performance in real time.
- **Adjust Parameters**: Use the dashboard controls to adjust temperature, humidity, light hours, and other parameters as needed.
- **Review Historical Data**: Access logged data to analyze past performance and make informed decisions about adjustments.

## Setup Instructions
1. **Deploy Software on Arduino**:

2. **AWS EC2 Deployment**:
   - Set up an AWS EC2 instance.
   - Deploy the Node-RED and MQTT servers on the EC2 instance to allow remote access to the dashboard.

3. **Database Setup**:
   - Install MySQL on your EC2 instance.
   - Import the provided database schema to set up the necessary tables and relationships.
  
4. **Deploy Software on D1 Mini, Editing Connection According to Your MQTT Server**:
   - Make sure to configure the Wi-Fi and MQTT server settings appropriately.

## Usage
- **Real-Time Monitoring**: Access the Node-RED dashboard to monitor environmental conditions and system performance in real time.
- **Adjust Parameters**: Use the dashboard controls to adjust temperature, humidity, light hours, and other parameters as needed.
- **Review Historical Data**: Access graphics and logged data to analyze performance.



