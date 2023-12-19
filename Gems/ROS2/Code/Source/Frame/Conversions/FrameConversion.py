# This script is used to convert ROS2FrameComponent to ROS2FrameEditorComponent in .prefab files.
# To use this script, run the following command:
# python script.py <directory_or_file>
# where <directory_or_file> is the directory or file to be processed.
# If <directory_or_file> is a directory, all .prefab files in the directory and its subdirectories will be processed.
# If <directory_or_file> is a file, only that file will be processed.

import sys
import simplejson as json
import os
import decimal

def find_and_replace(data):

    def search_for_components(item, foundComponents):
        if isinstance(item, dict):
            if "Components" in item:
                foundComponents = True
            if "ROS2FrameComponent" in item and foundComponents:
                modifiedElement = item["ROS2FrameComponent"]
                modifiedElement["$type"] = "ROS2FrameEditorComponent"
                del modifiedElement["m_template"]["$type"]
                modifiedElement["ROS2FrameConfiguration"] = modifiedElement["m_template"]
                del modifiedElement["m_template"]
                del item["ROS2FrameComponent"]
                item["ROS2FrameEditorComponent"] = modifiedElement
            if foundComponents and "m_template" in item:
                if "$type" in item["m_template"] and item["m_template"]["$type"] == "ROS2FrameComponent":
                    modifiedElement = item
                    modifiedElement["$type"] = "ROS2FrameEditorComponent"
                    del modifiedElement["m_template"]["$type"]
                    modifiedElement["ROS2FrameConfiguration"] = modifiedElement["m_template"]
                    del modifiedElement["m_template"]
                    item = modifiedElement
            if "op" in item and item["op"] == "replace" and "path" in item:
                item["path"] = item["path"].replace("ROS2FrameComponent/m_template", "ROS2FrameEditorComponent/ROS2FrameConfiguration")
            for value in item.values():
                search_for_components(value, foundComponents)
        elif isinstance(item, list):
            for element in item:
                search_for_components(element, foundComponents)

    search_for_components(data, False)


def write_json_to_file(data, filename):
    try:
        with open(filename, 'w') as file:
            json.dump(data, file, indent=4)
            print(f"Modified data written to '{filename}'.")
    except Exception as e:
        print(f"Error writing to '{filename}': {e}")

def find_prefab_files(directory):
    prefab_files = []
    if os.path.isfile(directory) and directory.endswith(".prefab"):
        prefab_files.append(directory)
    elif os.path.isdir(directory):
        for root, dirs, files in os.walk(directory):
            for file in files:
                if file.endswith(".prefab"):
                    prefab_files.append(os.path.join(root, file))
    return prefab_files

def parse_float_decimal(s):
    return decimal.Decimal(s)


class DecimalEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, decimal.Decimal):
            return str(obj)
        return json.JSONEncoder.default(self, obj)
    
def convert_scientific_notation(text):
    import re
    return re.sub(r'(\d+\.\d*|\.\d+)[Ee]([+-]?\d+)', r'\1e\2', text)


def process_prefab_file(file_path):
    try:
        with open(file_path, 'r') as file:
            data = json.load(file, parse_float=parse_float_decimal)
            find_and_replace(data)
        with open(file_path, 'w') as file:
            json.dump(data, file, indent=4)
            print(f"Modified data written to '{file_path}'.")
        content = ""
        with open(file_path, 'r') as file:
            content = file.read()
        content = convert_scientific_notation(content)
        with open(file_path, 'w') as file:
            file.write(content)
    except Exception as e:
        print(f"Error processing '{file_path}': {e}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <directory_or_file>")
    else:
        input_path = sys.argv[1]
        prefab_files = find_prefab_files(input_path)
        if not prefab_files:
            print(f"No .prefab files found in '{input_path}'.")
        else:
            for file_path in prefab_files:
                print(f"Processing '{file_path}'...")
                process_prefab_file(file_path)
