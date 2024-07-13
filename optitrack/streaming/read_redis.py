import redis
import os
import time
from datetime import datetime

# Redis configuration
REDIS_HOST = '127.0.0.1'
REDIS_PORT = 6379
OUTPUT_FILE = './recordings/simple1.txt'  # Single output file for all data

# Specific keys for rigid body positions and orientations
RIGID_BODY_POSITION_KEYS = [
    "sai2::optitrack::rigid_body_pos::1",
    "sai2::optitrack::rigid_body_pos::2",
    "sai2::optitrack::rigid_body_pos::3",
    "sai2::optitrack::rigid_body_pos::4",
    "sai2::optitrack::rigid_body_pos::5",
    "sai2::optitrack::rigid_body_pos::6"
]

RIGID_BODY_ORIENTATION_KEYS = [
    "sai2::optitrack::rigid_body_ori::1",
    "sai2::optitrack::rigid_body_ori::2",
    "sai2::optitrack::rigid_body_ori::3",
    "sai2::optitrack::rigid_body_ori::4",
    "sai2::optitrack::rigid_body_ori::5",
    "sai2::optitrack::rigid_body_ori::6"
]

ALL_KEYS = RIGID_BODY_POSITION_KEYS + RIGID_BODY_ORIENTATION_KEYS

# Connect to Redis
redis_client = redis.StrictRedis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=True)

def initialize_output_file():
    """
    Initialize the output file with a header row containing the keys.
    """
    with open(OUTPUT_FILE, 'w') as file:
        header = ['timestamp'] + ALL_KEYS
        file.write('\t'.join(header) + '\n')

def append_to_output_file(data):
    """
    Append a row of data to the output file.
    Each row includes a timestamp followed by the values for each key.
    """
    with open(OUTPUT_FILE, 'a') as file:
        row = [datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')] + [data.get(key, 'None') for key in ALL_KEYS]
        file.write('\t'.join(row) + '\n')

def read_and_append_keys():
    """
    Continuously read from the specified Redis keys and append their values to the output file.
    """
    initialize_output_file()
    while True:
        data = {}
        for key in ALL_KEYS:
            try:
                value = redis_client.get(key)
                if value is not None:
                    data[key] = value
            except redis.ConnectionError as e:
                print(f"Redis connection error: {e}")
                return
        append_to_output_file(data)
        time.sleep(1.0 / 120)  # Maintain a rate of 120 Hz

if __name__ == "__main__":
    read_and_append_keys()