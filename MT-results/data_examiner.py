import os
import zipfile
import json
import pandas as pd
import matplotlib.pyplot as plt
import argparse

def extract_zip(file_path):
    with zipfile.ZipFile(file_path, 'r') as zip_ref:
        zip_ref.extractall("data/")
    return [os.path.join("data", file) for file in os.listdir("data")]

def load_json_files(file_paths):
    data = {}
    for file_path in file_paths:
        with open(file_path, 'r') as file:
            data[file_path] = [json.loads(line) for line in file]
    return data

def process_entry(entry):
    processed = {}
    processed['Reverse'] = False
    processed['Hand brake'] = False
    processed['Manual'] = False

    for item in entry:
        if isinstance(item, str):
            parts = item.split(":")
            if len(parts) == 2:
                key, value = parts
                key = key.strip()
                value = value.strip()

                if key in ['Speed', 'Accelero', 'Gyroscop']:
                    numeric_value = float(''.join(filter(str.isdigit, value)))/10 if value else 0
                    processed[key] = numeric_value
                elif key == 'Gear':
                    processed[key] = value
        elif isinstance(item, list):
            if len(item) == 2:
                key, value = item
                key = key.strip()

                if key in ['Reverse', 'Hand brake', 'Manual']:
                    processed[key] = value

    return processed

def process_data(data):
    return {file_name: [process_entry(entry) for entry in file_data] for file_name, file_data in data.items()}

def data_to_df(data):
    return {file_name: pd.DataFrame(file_data) for file_name, file_data in data.items()}

def compute_statistics(df_dict):
    statistics = {}
    for file_name, df in df_dict.items():
        print(f"File: {file_name}")
        print("\nDescriptive Statistics:")
        print(df[['Speed', 'Accelero', 'Gyroscop']].describe())
        print("\nBoolean Counts:")
        print(df[['Reverse', 'Hand brake', 'Manual']].apply(pd.Series.value_counts))
        print("\n\n")

def plot_variable(df_dict, variable):
    fig, axs = plt.subplots(2, len(df_dict), figsize=(18, 10))

    for ax, (file_name, df) in zip(axs.T, df_dict.items()):
        ax[0].hist(df[variable], bins=20, color='skyblue', edgecolor='black')
        ax[0].set_title(f'Histogram of {variable} - {file_name}')

        ax[1].boxplot(df[variable].dropna(), vert=False)
        ax[1].set_title(f'Boxplot of {variable} - {file_name}')

    plt.tight_layout()
    plt.show()

# Create an argument parser
parser = argparse.ArgumentParser(description="Analyze driving data in zip file.")
parser.add_argument('-f', '--file', help="Path to the zip file.", required=True)

# Parse command-line arguments
args = parser.parse_args()

# Extract zip file and get list of JSON files
json_file_paths = extract_zip(args.file)

# Load and process the JSON data
raw_data = load_json_files(json_file_paths)
processed_data = process_data(raw_data)

# Convert the processed data to pandas DataFrames
df_dict = data_to_df(processed_data)

# Compute and print descriptive statistics
compute_statistics(df_dict)

# Plot histograms and boxplots
for variable in ['Speed', 'Accelero', 'Gyroscop']:
    plot_variable(df_dict, variable)
