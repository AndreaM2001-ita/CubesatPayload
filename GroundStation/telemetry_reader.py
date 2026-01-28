# Telemetry Reader
# Subscribes to the mqtt broker on  the Raspberry Pi
# Parses the JSON file for the telemetry data and image file
# Append telemetry data to csv file and perform cloud detection 

#!/usr/bin/env python3
import csv				# CSV library to append telemetry data to csv file
import random			# Create random client id
import time				# For timestamp
import json				# For parsing/dumping json file
import pathlib			# To access file system paths
import os				# For  file handling
import cv2				# OpenCV for image reading and conversion
import base64			# For encoding/decoding base64 image string
import numpy as np 		# Mathematical functions
import yaml				# Library to access opencv config stored in YAML file
import sys				
import socket
from imageAnalysis import find_mask_cloud	# Import image processing function 
from paho.mqtt import client as mqtt_client	# Establish ground station as mqtt client device

# Define the MQTT broker details
#broker = "192.168.190.25"   	# Fixed local IP address
broker  = "localhost"

print(broker)
port = 1883
topic = "sat/telemetry/#"		# subscribe to all topic on /telemetry topic level
client_id = f'subscribe-{random.randint(0, 100)}'


root = pathlib.Path.home()/ "groundstation"

# ---------- MQTT helpers ----------

def connect_mqtt() -> mqtt_client:
	def on_connect(client, userdata, flags, rc):
		if rc == 0:
			print("Connected to MQTT broker")
		else:
			print("Failed to connect, return code %d\n", rc)
	client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1, client_id)
	client.on_connect = on_connect
	client.connect(broker, port)
	return client

def subscribe(client: mqtt_client):
	def on_message(client, userdata, msg):
		try:
			print(f"Received message from '{msg.topic}' topic\n")
			

			print(msg.payload)
			# Decode JSON file int python object
			payload = msg.payload.decode('utf-8')
			data = json.loads(payload)
		
			

			# Append data to CSV only when receive message from fusion node
			# Separate timestamp for remote command to avoid conflict with scheduled data 
			if msg.topic == "sat/telemetry/DataReady":
				# Create timestamp for image output file name
				timestamp = time.ctime(data["ts"])

				# Append telemetry data to csv file
				fusion_csv(data)
				
				if "Image" in data:
					imagebase64 = data["Image"]
					convert_image(timestamp, imagebase64)
				else:
					print("No image data in payload")
			elif msg.topic == "sat/telemetry/RemoteReady":

				# Create timestamp for remote command image output file name
				timestampRemote = time.ctime(data["ts"])
				if "Image" in data:
					imagebase64Remote = data["Image"]
					convert_image(timestampRemote, imagebase64Remote)
				else:
					print("No image data in payload")
			else:
				print(f"Ignored message from {msg.topic}")

		except json.JSONDecodeError:
			print("Error: Failed to decode JSON from the payload")
		except Exception as e:
			print("Unexpected error: ")
			print(e)
	client.subscribe(topic)
	client.on_message = on_message


def run():

	client = connect_mqtt()
	subscribe(client)
	client.loop_forever()


# Convert image from base64 string to jpeg
# Save file in current groundstation folder
def convert_image(timestamp, image):
    try:

		# Designate timestapm as filename for cloud image
        filename = f'{timestamp}.jpg'
      
        # Decode and save raw bytes for inspection
        img_bytes = base64.b64decode(image)
        with open(filename, 'wb') as f:
            f.write(img_bytes)
        print(f"Saved  image to {filename} (size: {len(img_bytes)} bytes)")
        image = cv2.imread(filename)
        find_mask_cloud(image,timestamp)

    except Exception as e:
        print(f"Error during image conversion: {e}")

# Append sensor reading into csv file
def fusion_csv(payload_data):

	# Grab values from json payload
	values = payload_data["values"]
	csv_data = [{
		"Timestamp": time.ctime(payload_data["ts"]),
		"Confidence": values["confidence"],
		"Presence": values["presence"],
		"Temperature": values["temp"],
		"Humidity": values["hum"],
		"Pressure": values["press"],
		"Light": values["light"]}]


	filename = 'payload_records.csv'
	with open(filename, 'a', newline='') as csvfile:
		fieldnames = ['Timestamp', 'Confidence', 'Presence', 'Temperature', "Humidity", "Pressure", "Light"]
		writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
		if not (os.path.exists(filename)):
			writer.writeheader()
		writer.writerows(csv_data)
	
	print(f"Telemetry Data Appended to {filename}")
	

if __name__ == '__main__':
	run()



