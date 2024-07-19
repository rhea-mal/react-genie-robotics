import redis
import time
import csv
import argparse
from datetime import datetime, timedelta

# Redis configuration
REDIS_HOST = '127.0.0.1'
REDIS_PORT = 6379
INPUT_FILE = '/Users/rheamalhotra/Desktop/robotics/react-genie-robotics/optitrack/recordings/realsense.txt'  # Input file with the data

# Connect to Redis
redis_client = redis.StrictRedis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=True)

def read_data(file_path):
    """
    Read data from the input file and return it as a list of dictionaries.
    Each dictionary represents a row with key-value pairs.
    """
    data = []
    try:
        with open(file_path, 'r') as file:
            reader = csv.DictReader(file, delimiter='\t')
            for row in reader:
                data.append(row)
    except FileNotFoundError:
        print(f"File not found: {file_path}")
    except Exception as e:
        print(f"An error occurred while reading the file: {e}")
    return data

def publish_to_redis(data, rate_hz):
    """
    Publish each row of data to Redis at the specified rate.
    """
    interval = 1.0 / rate_hz
    while True:
        start_time = datetime.now()
        for row in data:
            timestamp = row.pop('timestamp', None)
            current_time = datetime.now() if timestamp is None else datetime.strptime(timestamp, '%Y-%m-%d %H:%M:%S.%f')
            for key, value in row.items():
                redis_client.set(key, value)
            time.sleep(interval)
        elapsed_time = (datetime.now() - start_time).total_seconds()
        if elapsed_time < interval * len(data):
            time.sleep(interval * len(data) - elapsed_time)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Redis Data Publisher')
    args = parser.parse_args()

    data = read_data(INPUT_FILE)
    if data:
        try:
            publish_to_redis(data, rate_hz=120)
        except KeyboardInterrupt:
            print("\nScript interrupted. Exiting...")
