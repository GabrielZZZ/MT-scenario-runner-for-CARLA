import json
from math import sqrt

# Function to parse the location and GNSS data
def parse_location_gnss(record):
    location, gnss = None, None
    for line in record:
        if isinstance(line, str):
            if line.startswith('Location:'):
                location = tuple(map(float, line.replace('Location:', '').strip().strip('()').split(',')))
            elif line.startswith('GNSS:'):
                gnss = tuple(map(float, line.replace('GNSS:', '').strip().strip('()').split(',')))
    return location, gnss

# Load the data from the file, line by line
def load_and_parse_data(file_path):
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            data.append(json.loads(line))
    
    # Extract the 'Location' and 'GNSS' data from each record
    locations_gnss = [parse_location_gnss(record) for record in data]
    return locations_gnss

# Function to calculate Euclidean distance between two points
def euclidean_distance(point1, point2):
    return sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

# Calculate the Euclidean distance between the 'Location' or 'GNSS' points in two datasets
def calculate_distances(locations_gnss_1, locations_gnss_2, data_type='Location'):
    if data_type == 'Location':
        index = 0
    elif data_type == 'GNSS':
        index = 1
    distances = [euclidean_distance(point1[index], point2[index]) for point1, point2 in zip(locations_gnss_1, locations_gnss_2)]
    return distances

# Usage:
file_path_1 = 'MT-scenario_runner-0.9.13/MT-results/info_text_20230730-213555.json'
file_path_2 = 'MT-scenario_runner-0.9.13/MT-results/info_text_20230730-213645.json'

locations_gnss_1 = load_and_parse_data(file_path_1)
locations_gnss_2 = load_and_parse_data(file_path_2)

distances_location = calculate_distances(locations_gnss_1, locations_gnss_2, data_type='Location')
distances_gnss = calculate_distances(locations_gnss_1, locations_gnss_2, data_type='GNSS')

print(f"Mean Euclidean distance between 'Location' points: {sum(distances_location)/len(distances_location)}")
print(f"Mean Euclidean distance between 'GNSS' points: {sum(distances_gnss)/len(distances_gnss)}")