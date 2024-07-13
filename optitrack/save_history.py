import redis
import time
import csv
from datetime import datetime, timedelta

# Redis configuration
REDIS_HOST = '127.0.0.1'
REDIS_PORT = 6379
OUTPUT_FILE = '/Users/rheamalhotra/Desktop/robotics/react-genie-robotics/optitrack/recordings/history.txt'  # Output file for the data

# Connect to Redis
redis_client = redis.StrictRedis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=True)

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

history = {key: [] for key in ALL_KEYS}

def cleanup_old_entries(history, current_time):
    """
    Remove entries older than 30 seconds from the history.
    """
    cutoff_time = current_time - timedelta(seconds=30)
    for key in list(history.keys()):
        history[key] = [entry for entry in history[key] if entry['timestamp'] >= cutoff_time]
        if not history[key]:
            del history[key]

def initialize_output_file():
    """
    Initialize the output file with a header row containing the keys.
    """
    with open(OUTPUT_FILE, 'w') as file:
        header = ['timestamp'] + ALL_KEYS
        file.write('\t'.join(header) + '\n')

def append_to_output_file(history):
    """
    Append rows of data from the history to the output file.
    Each row includes a timestamp followed by the values for each key.
    """
    with open(OUTPUT_FILE, 'a') as file:
        for timestamp in sorted(set(entry['timestamp'] for key in history for entry in history[key])):
            row = [timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')] + [next((entry['value'] for entry in history[key] if entry['timestamp'] == timestamp), 'None') for key in ALL_KEYS]
            file.write('\t'.join(row) + '\n')

def read_and_append_keys():
    """
    Continuously read from the specified Redis keys and append their values to the output file.
    """
    initialize_output_file()
    while True:
        current_time = datetime.now()
        data = {}
        for key in ALL_KEYS:
            try:
                value = redis_client.get(key)
                if value is not None:
                    if key not in history:
                        history[key] = []
                    history[key].append({'timestamp': current_time, 'value': value})
            except redis.ConnectionError as e:
                print(f"Redis connection error: {e}")
                return

        cleanup_old_entries(history, current_time)
        append_to_output_file(history)
        time.sleep(1.0 / 120)  # Maintain a rate of 120 Hz

if __name__ == "__main__":
    read_and_append_keys()