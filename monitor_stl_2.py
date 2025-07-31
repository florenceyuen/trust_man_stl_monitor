"""
STL (Signal Temporal Logic) specifies the time-dependent behaviours of systems
Formulas describe how signals should behave over time to help monitor the values
This includes speed, acceleration, distance

Uses rtamt package which defines STL formulas, applies onto signals, and returns the robustness
No ros version

pip install rtamt: installation
"""

# Import and initialize a specification
import rtamt
import logging
import sys
import os
import json

import csv
from properties import Property
LOG_LEVEL = logging.DEBUG

# list of stl proprties and their formulas
STL_PROPERTY_FORMULAS = {
    "speed": {
        "id": 0,
        "formula": "out = always(("
        "  (speed > 100)"
        ") implies eventually[0:5]("
        "  speed <= 100"
        "))",
    },
    "acc": {
        "id": 1,
        "formula": "out=always[0:5](acceleration >= -20 and acceleration <=20)",
    },
    "xpos": {
        "id": 2,
        "formula": "out = always(("
        "  (x < 0 or x > 600)"
        ") implies eventually[0:5]("
        "  x >= 0 and x <= 600"
        "))",
    },
    "all": {
        "id": 3,
        "formula": "out = always(("
        "  (speed > 100 or acceleration < -20 or acceleration > 20 or x < 0 or x > 600)"
        ") implies eventually[0:5]("
        "  speed <= 100 and acceleration >= -20 and acceleration <= 20 and x >= 0 and x <= 600"
        "))",
    },
}

stl_thresholds = {
    "speed": {"max": 100},  # speed (m/s)
    "acceleration": {"min": -20, "max": 20},  # for violation: -5},
    "x": {"min": 0, "max": 600},  # x position
    "y": {"min": 0, "max": 500},  # y position
    "angle": {"min": 0, "max": 20},  # steering angle
    "window_time": 5,  # seconds for each violation window
}


class STLMonitor:
    """
    Base STL monitor class used by discrete and dense stl specifications
    """

    def __init__(self):
        # Define the STL specification
        self.spec = self.init_spec()

    def init_spec(self):
        raise NotImplementedError("Child class must override init_spec function")

    def load_data_from_csv(self, filename="discrete_stl_data.csv"):
        """
        Dynamically read the csv column names
        """

        data = {}  # dict to store data based on column name

        # Reads the timestamp and variable data from CSV, returns as list of tuples
        try:
            with open(filename, mode="r") as file:
                reader = csv.DictReader(file)
                headers = (
                    reader.fieldnames
                )  # get the list of column names from csv file

                if not headers:
                    print(f"No headers found in '{filename}'")
                    return {}

                if "time" not in headers:  # make sure there is a time column
                    print(f"'time' column missing from '{filename}'")
                    return {}

                # initialize lists for each column in csv file except 'time'
                for header in headers:
                    if header != "time":
                        data[header] = []

                for row in reader:
                    try:
                        time = float(row["time"])
                        for header in data:
                            value = float(row[header])
                            data[header].append((time, value))
                    except (ValueError, KeyError) as e:
                        print(f"Invalid row {row}, skipping: {e}")
        except FileNotFoundError:
            print(f"File '{filename}' not found.")
            return {}

        except Exception as e:
            print(f"Error reading '{filename}': {e}")
            return {}

        return data


class DiscreteSTLMonitor(STLMonitor):
    def init_spec(self):
        # Define the STL specification
        spec = rtamt.StlDiscreteTimeSpecification()
        spec.name = "Car Speed Monitor (Discrete Time)"

        # Declare input/ output signals (speed, acceleration)
        spec.declare_var("speed", "float")  # Input signal
        spec.declare_var("acc", "float")  # Accerleration
        spec.declare_var("pos", "float")  # Position
        spec.declare_var("out", "float")  # Output signal (formula result)
        spec.set_var_io_type("out", "output")
        spec.set_var_io_type("speed", "input")
        spec.set_var_io_type("acc", "input")
        spec.set_var_io_type("pos", "input")

        # Write STL formula
        # self.spec.spec = 'out = always[0:5](speed <= 100)' # speed is always less than 100 for the first 5 seconds
        spec.spec = (
            "out = always[0,5]("
            "speed <= 100 and "  # speed is always less than 100 for the first 30 seconds
            "acc >= -20 and acc <= 20 and "
            "pos >= 0 and pos <= 600)"
        )

        # Parse the formula to make it ready to use
        spec.parse()
        return spec

    def evaluate_single_prop(self, speed_data, acc_data, pos_data):

        time_data = [t for t, _ in speed_data]  # extract time list
        speed_values = [v for _, v in speed_data]
        acc_values = [v for _, v in acc_data]
        pos_values = [v for _, v in pos_data]

        print("time_data [0:3] samples:", time_data[:3])
        print("Speed_values [0:3] samples:", speed_values[:3])
        print("acc_values [0:3] samples:", acc_values[:3])
        print("pos_values [0:3] samples:", pos_values[:3])

        dataset = {
            "time": time_data,
            "speed": speed_values,
            "acc": acc_values,
            "pos": pos_values,
        }

        print("Dataset keys:", dataset.keys())

        # try:
        result = self.spec.evaluate(dataset)
        # except Exception as e:
        #     print(f"Error during evaluation: {e}")
        #     return

        print("Time\tRobustness")
        for timestamp, robustness in result:
            status = "success" if robustness >= 0 else "violation"
            print(f"Time {timestamp}s: robustness={robustness:.2f} → {status}")


class DenseSTLMonitor(STLMonitor):
    """
    STL monitor using dense time specification
    """
    def __init__(self, logger=None):    
        self.logger=logger
        self.name = 'Car Speed Monitor (Dense Time)'

    def define_spec(self, data):
        # Define the STL specification
        spec = rtamt.StlDenseTimeSpecification()
        spec.name = self.name
        
        # Declare variables dynamically from Property class
        for var, dtype in Property.headings.items():
            if var == "time" or var == "device_id":  # Skip time since rtamt handles it separately and device_id
                continue

            # Use type for each property
            spec.declare_var(var, dtype)
            spec.set_var_io_type(var, 'input')

            self.logger.info(f"Declared variable: {var} ({dtype})")

        # Declare output signal
        spec.declare_var('out', 'float')    # Output signal (formula result)
        spec.set_var_io_type('out', 'output')
        
        # Dynamically declare all input variables read from csv file
        stl_formula = self.dyn_build_formula(data)

        # Write STL formula (all properties)
        # spec.stl_formula = STL_PROPERTY_FORMULAS["all"]["formula"]
        # first 15 seconds, initial positions and swap on x and y axis for benchmark, position always halfway mark, and then toggle when change platoon leader

        # STL formulas for two properties (e.g. speed, position), simultaneously check violations
        stl_formula2 = STL_PROPERTY_FORMULAS["speed"]["formula"]
        
        # Write default STL formula
        spec.spec = (
            stl_formula2
        )

        # Parse the formula to make it ready to use
        spec.parse()
        self.spec = spec
        
    def evaluate_all_signals(self, data, vehicle_id=0, output_file='stl_results/stl_results.json'): 
        """
        Evaluate all stl properties listed in dictionary
        """

        results_list = []
        # Evaluate every STL property
        for property_name, property_info in STL_PROPERTY_FORMULAS.items():
            try:
                # Update spec name
                self.spec.name = f"{self.name} - {property_name}"

                # Display the which property is being evaluated, robustness values and whether property was violated
                if self.logger is not None:
                    self.logger.info(f"{self.spec.name}")
                    self.logger.info(f"Time\tRobustness")

                # Assign formula for that property
                self.spec.spec = property_info['formula']
                self.spec.parse()

                results_list.append(self.evaluate_single_prop(data, property_info['id'], vehicle_id, output_file))
            except Exception as e:
                if self.logger:
                    self.logger.error(f"Failed to evaluate STL property '{property_name}' id: {property_info['id']}: {e}")
        
        return results_list

    def evaluate_single_prop(self, data, spec_id=0, vehicle_id=0, output_file='stl_results/stl_results.json'): 
        """
        Evaluates one stl property and appends to output json
        """
        # Evaluate the signal against the STL rule/ formula
        input_signals = []
        for var, value in data.items():
            input_signals.append([var, value])
            
        try:
            result = self.spec.evaluate(*input_signals) # use unpacking operator
        except Exception as e:
            if self.logger is not None:
                self.logger.error(f"Failed to evaluate STL (car {vehicle_id}): {e}")
            return []
        
        # Update so runs on memory
        results_list=[]

        # TODO: Output last 10 seconds to separate location, temp write to a file
        for timestamp, robustness in result:
            status = "success" if robustness >= 0 else "violation"

            # Add timestamp entry and satisfaction to JSON list
            results_list.append({
                "timestamp": float(timestamp),
                "vehicle_id": int(vehicle_id),
                "stl_spec_id": int(spec_id),    # index of stl property from dict
                "robustness": float(robustness), # stl property id, add rosbag and eval on that 
                "status": status,
            })

            self.logger.info(f"[car-{vehicle_id}] Time={timestamp}s: robustness={robustness:.2f} → {status}")   

        self.logger.info(f"\n")
        return results_list
    
    def write_json(self, results_list, output_file='stl_result/stl_results.json'):
        # Load the existing data if the file already exists
        if os.path.exists(output_file):
            with open(output_file, 'r') as file:
                existing_data = json.load(file)
        else:
            existing_data=[]

        existing_data.extend(results_list)

        try:
            with open(output_file, 'w') as file:
                json.dump(existing_data, file, indent=2)
                self.logger.info(f"STL result saved to {output_file}")
        except Exception as e:
            self.logger.error(f"Error writing JSON file: {e}")
    
    def dyn_build_formula(self, data):
        violations = []
        goal = []

        for var, rules in stl_thresholds.items():
            if var == 'window_time':
                continue
            if var not in data:
                self.logger.warn(f"Warning: Variable '{var}' is missing from csv file, skipping in formula")
                continue
            if 'min' in rules:
                violations.append(f'{var} < {rules["min"]}')
                goal.append(f'{var} >= {rules["min"]}')
            if 'max' in rules:
                violations.append(f'{var} > {rules["max"]}')
                goal.append(f'{var} <= {rules["max"]}')
        
        violation_formula = ' or '. join(violations)
        recovery_formula = ' and '.join(goal)
        window_time = stl_thresholds.get('window_time', 5) # fallback default value in case not defined

        stl_formula = f'out = always(({violation_formula}) implies eventually[0:{window_time}]({recovery_formula}))'

        self.logger.info(f"STL formula: {stl_formula}")
        return stl_formula     
    
# class MonitorNode(Node):
class MonitorNode:
    MONITOR_TYPE="dense" # default is dense time
    def __init__(self, logger, filename, monitor_type=MONITOR_TYPE):  # default is discrete time
        self.logger=logger
        self.logger.info("Monitor Node started.\n")
        self.monitor_type=monitor_type 

        self.logger.info(f"Using {self.monitor_type} Time Specification")

         # Select which STL monitor to use (discrete or dense)
        if self.monitor_type == 'dense':
            self.monitor = DenseSTLMonitor(self.logger)
        else:
            self.monitor = DiscreteSTLMonitor()
            
        self.logger.info(f"Opening file path: {filename}")
        data = self.monitor.load_data_from_csv(filename)
        
        self.spec = self.monitor.define_spec(data)

        # Feed in test signal data for speed (time, value), acceleration, and position
        results_list = []
        results_list = self.monitor.evaluate_all_signals(data)
        
        self.output_file_path = 'stl_result/stl_results.json'
        
        # Create the parent directory if it doesn't already exist
        os.makedirs(os.path.dirname(self.output_file_path), exist_ok=True)

        # Create file if doesn't exist and overwrite if already exists
        with open(self.output_file_path, 'w') as file:
            json.dump([], file, indent=2) # Initialize file with empty list
        self.monitor.write_json(results_list)

def init_logger():
    # Initialize the log handler
    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(LOG_LEVEL)

    # Specify the log format
    formatter = logging.Formatter("%(asctime)s %(levelname)s [%(filename)s:%(lineno)d] %(message)s")
    handler.setFormatter(formatter)

    # Initialize the root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(LOG_LEVEL)
    root_logger.addHandler(handler)
    return root_logger

def main(args=None):
    logger = init_logger()
    filename = "discrete_stl_data.csv"
    
    MonitorNode(logger, filename)


if __name__ == "__main__":
    main()
