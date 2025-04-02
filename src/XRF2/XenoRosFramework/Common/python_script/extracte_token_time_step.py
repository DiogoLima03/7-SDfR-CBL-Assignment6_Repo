import xml.etree.ElementTree as ET
import argparse

# Set up argument parsing
parser = argparse.ArgumentParser(description='Extract TIME_STEP_SIZE from XML file.')
parser.add_argument('xml_file', type=str, help='The path to the XML file')

args = parser.parse_args()

# Parse the XML file
tree = ET.parse(args.xml_file)
root = tree.getroot()

# Find the token with the name 'TIME_STEP_SIZE'
time_step_token = root.find('./token[@name="TIME_STEP_SIZE"]')

# Extract and print the value
if time_step_token is not None:
    time_step_value = time_step_token.text.strip()
    print(time_step_value)  # This will be captured by CMake
else:
    print("TIME_STEP_SIZE token not found")
