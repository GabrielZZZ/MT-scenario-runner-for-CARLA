import os
import subprocess
import json
import random
import argparse
import xml.etree.ElementTree as ET
from copy import deepcopy
import ast
import astunparse

# List of available vehicle models
vehicle_models = [
    'vehicle.audi.a2',
    'vehicle.audi.tt',
    'vehicle.audi.etron',
    'vehicle.bmw.grandtourer',
    'vehicle.bmw.is3',
    'vehicle.bmw.x5',
    'vehicle.mercedes-benz.coupe',
    'vehicle.toyota.prius',
    'vehicle.yamaha.yzf',
    'vehicle.dodge_charger.police'
]

# Original parameters and values
parameters = [
    ("first_actor_location", 25),
    ("second_actor_location", 40),
    ("first_actor_speed", 10),
    ("second_actor_speed", 10),

    ("other_actor_max_brake", 1.0),
    ("other_actor_stop_in_front_intersection", 20)
    # ... (add other parameters as needed)
]

def run_scenario_runner(scenario, f_index):
    additional_arg = ""
    if f_index == 0:
        additional_arg = "--reloadWorld"  
    
    if f_index == 0:
        command = f"python3.8 scenario_runner.py --scenario {scenario} {additional_arg} --output"
    else:
        command = f"python3.8 scenario_runner.py --scenario {scenario + '_f' + str(f_index+1)} {additional_arg} --output"
    
    return subprocess.Popen(command, shell=True)

def run_manual_control():
    command = "python3.8 manual_control.py"
    return subprocess.Popen(command, shell=True)

def create_scenario(root, old_scenario_name, scenario_name, config=None):

    # Find an existing scenario from the given scenarios
    template_scenario = root.find(f'scenario[@name="{old_scenario_name}"]')

    if template_scenario is None:
        print(f"No existing scenarios found!")
        return

    # Create a new scenario and copy the ego_vehicle from the template
    scenario = ET.Element('scenario')
    scenario.set('name', scenario_name)
    scenario.set('type', template_scenario.attrib['type'])
    scenario.set('town', template_scenario.attrib['town'])
    
    ego_vehicle_template = template_scenario.find('ego_vehicle')
    ego_vehicle = ET.SubElement(scenario, ego_vehicle_template.tag, attrib=ego_vehicle_template.attrib)

    parameters_elem = ET.SubElement(scenario, 'parameters')

    # Get order of parameters from template scenario
    template_parameters = template_scenario.find('parameters').attrib
    for parameter in template_parameters:
        value = str(config[parameter]) if config and parameter in config else template_parameters[parameter]
        parameters_elem.set(parameter, value)

    root.append(scenario)

def get_starting_index(root, scenario_name):
    import re

    # Regular expression pattern for scenario names
    pattern = re.compile(f"{scenario_name}_f(\\d+)")

    # Find all scenario names in the XML file
    scenario_names = [element.get('name') for element in root.findall('scenario')]

    # Find all scenario names that match the pattern
    matching_scenario_names = [name for name in scenario_names if pattern.match(name)]

    # Extract the integer suffixes and find the maximum
    suffixes = [int(pattern.match(name).group(1)) for name in matching_scenario_names]
    max_suffix = max(suffixes) if suffixes else 0

    return max_suffix + 1


def follow_directions(xml_file, scenarios, scenario_count, mode):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    parameters.sort(key=lambda x: x[1])
    
    config_list = []

    for index, scenario_name in enumerate(scenarios):
        template_scenario = root.find(f'scenario[@name="{scenario_name}"]')
        if template_scenario is not None:
            starting_index = get_starting_index(root, scenario_name)
            for i in range(starting_index, starting_index + scenario_count):
                config = {}
                if mode == "d1":
                    # Completely randomize parameters
                    for param in parameters:
                        min_value = 0.3 * param[1]
                        max_value = 2 * param[1]
                        random_value = random.uniform(min_value, max_value)
                        if param[0].endswith("_ratio"):
                            random_value = min(random_value, 1)
                        config[param[0]] = random_value
                    config_list.append(config)
                elif mode == "d2":
                    # Only decrease "second_actor_speed"
                    template_parameters = template_scenario.find('parameters').attrib
                    for param in template_parameters:
                        if param == "second_actor_speed":
                            min_value = 0.3 * float(template_parameters[param])
                            max_value = float(template_parameters[param])
                            random_value = random.uniform(min_value, max_value)
                            config[param] = random_value
                        else:
                            config[param] = template_parameters[param]
                    
                new_scenario_name = f"{scenario_name}_f{i}"
                create_scenario(root, scenario_name, new_scenario_name, config)
        else:
            print(f"Scenario {scenario_name} not found!")

    tree.write(xml_file)



def change_vehicle_type(xml_file, scenarios, scenario_count):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    for index, scenario_name in enumerate(scenarios):
        template_scenario = root.find(f'scenario[@name="{scenario_name}"]')
        if template_scenario is not None:
            starting_index = get_starting_index(root, scenario_name)
            for i in range(starting_index, starting_index + scenario_count):
                selected_vehicles = random.sample(vehicle_models, 2)
                config = {'first_vehicle_type': selected_vehicles[0], 'second_vehicle_type': selected_vehicles[1]}
                
                new_scenario_name = f"{scenario_name}_f{i}"
                create_scenario(root, scenario_name, new_scenario_name, config)
        else:
            print(f"Scenario {scenario_name} not found!")

    tree.write(xml_file)


def merge_classes(class_name_1, class_name_2, output_file):
    # Read the Python file
    with open(output_file, 'r') as file:
        code = file.read()

    # Parse the code into an abstract syntax tree (AST)
    code_ast = ast.parse(code)

    # Extract the class definitions
    class_1_def = next((node for node in code_ast.body if isinstance(node, ast.ClassDef) and node.name == class_name_1), None)
    class_2_def = next((node for node in code_ast.body if isinstance(node, ast.ClassDef) and node.name == class_name_2), None)

    if class_1_def is None or class_2_def is None:
        print(f"Could not find the classes {class_name_1} and/or {class_name_2} in the file {output_file}!")
        return

    # Create a new combined class definition
    combined_class_def = ast.ClassDef(
        name='FollowLeadingVehicleWithSideVehicleCombined',
        bases=[],
        body=[],
        decorator_list=[],
        keywords=[]
    )

    # Add methods and attributes from the first class
    for node in class_1_def.body:
        combined_class_def.body.append(deepcopy(node))

    # Add methods and attributes from the second class, if they don't conflict with the first class
    for node in class_2_def.body:
        if not any(isinstance(n, type(node)) and n.name == node.name for n in class_1_def.body):
            combined_class_def.body.append(deepcopy(node))

    # Add the combined class to the existing code
    code_ast.body.append(combined_class_def)

    # Unparse the AST back into code
    combined_code = astunparse.unparse(code_ast)

    # Write the modified code back into the Python file
    with open(output_file, 'w') as file:
        file.write(combined_code)


def copy_scenario(xml_file, scenario_name, class_name, new_class_name):
    # Parse the XML file
    tree = ET.parse(xml_file)
    root = tree.getroot()

    # Find a scenario that uses one of the classes provided
    scenario = root.find(f'scenario[@type="{class_name}"]')
    if scenario is None:
        print(f"No scenario found that uses the class {class_name}!")
        return

    # Copy the scenario and modify it to use the new combined class
    new_scenario = deepcopy(scenario)
    new_scenario.set('name', scenario_name)
    new_scenario.set('type', new_class_name)

    # Add the new scenario to the XML file
    root.append(new_scenario)

    # Write the modified XML back to the file
    tree.write(xml_file)

def main():
    parser = argparse.ArgumentParser(description='Test harness for scenario runner.')
    parser.add_argument('-r', action='store_true', help='Run the scenario execution process')
    parser.add_argument('-g', nargs='*', type=str, help='Generate test cases')
    parser.add_argument('-n', type=int, default=1, help='Number of scenarios to generate')
    parser.add_argument('-d1', action='store_true', help='Completely randomize parameters')
    parser.add_argument('-d2', action='store_true', help='Only decrease second_actor_speed')
    # ...
    parser.add_argument('-ctype', action='store_true', help='Randomly choose vehicle types and save to a config file')
    parser.add_argument('-s', '--scenario', type=str, required=False, help='The scenario to run')
    parser.add_argument('-xml', type=str, required=False, help='The XML file to modify')
    parser.add_argument('-t', '--type', type=str, required=False, help='Type of scenarios to create')
    parser.add_argument('-f', type=int, default=1, help='Number of follow-up scenarios to run')
    parser.add_argument('-m', '--merge_classes', nargs=2, help='Merge two classes into a new combined class')



    args = parser.parse_args()

    if args.g:
        if args.d1:
            follow_directions(args.xml, args.g, args.n, "d1")
            
        elif args.d2:
            follow_directions(args.xml, args.g, args.n, "d2")

        if args.ctype:
            change_vehicle_type(args.xml,  args.g, args.n)

    if args.r:
        for f_index in range(args.f):
            print(f"Running iteration {f_index+1}...")
            scenario_runner_process = run_scenario_runner(args.scenario, f_index)
            manual_control_process = run_manual_control()

            scenario_runner_process.communicate()
            manual_control_process.communicate()

            print(f"Iteration {f_index+1} finished.")

if __name__ == '__main__':
    main()

