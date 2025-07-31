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
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Float32MultiArray

import csv
from .properties import Property

# list of stl proprties and their formulas
STL_PROPERTY_FORMULAS = {
    "speed": {
        "id": 0,
        "formula": 'out = always(('
            '  (speed > 100)'
            ') implies eventually[0:5]('
            '  speed <= 100'
            '))',
    },
    "acc": {
        "id": 1,
        "formula": "out=always[0:5](acceleration >= -20 and acceleration <=20)",
    },
    "xpos": {
        "id": 2,
        "formula": 'out = always(('
            '  (x < 0 or x > 600)'
            ') implies eventually[0:5]('
            '  x >= 0 and x <= 600'
            '))'
    },
    "all": {
        "id": 3,
        "formula": 'out = always(('
            '  (speed > 100 or acceleration < -20 or acceleration > 20 or x < 0 or x > 600)'
            ') implies eventually[0:5]('
            '  speed <= 100 and acceleration >= -20 and acceleration <= 20 and x >= 0 and x <= 600'
            '))',
    },
}

stl_thresholds={
    'speed': {'max': 100},  # speed (m/s)
    'acceleration':{'min': -20,'max': 20}, # for violation: -5},
    'x':{'min': 0,'max': 600},  # x position
    'y':{'min': 0,'max': 500},  # y position
    'angle': {'min': 0, 'max': 20},  # steering angle
    'window_time': 5, # seconds for each violation window
}

class STLMonitor():
    """
    Base STL monitor class used by discrete and dense stl specifications
    """
    def __init__(self):    
        # Define the STL specification
        self.spec = self.init_spec()
        
    def init_spec(self):
        raise NotImplementedError("Child class must override init_spec function")    
            
    def load_data_from_csv(self, filename='discrete_stl_data.csv'):
        """
        Dynamically read the csv column names
        """

        data = {} # dict to store data based on column name

        # Reads the timestamp and variable data from CSV, returns as list of tuples
        try:
            with open(filename, mode='r') as file:
                reader = csv.DictReader(file)
                headers = reader.fieldnames # get the list of column names from csv file

                if not headers: 
                    print(f"No headers found in '{filename}'")
                    return {}

                if 'time' not in headers: # make sure there is a time column
                    print(f"'time' column missing from '{filename}'")
                    return {}
                
                # initialize lists for each column in csv file except 'time'
                for header in headers:
                    if header != 'time':
                        data[header] = []

                for row in reader:
                    try:
                        time = float(row['time'])
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
        spec.name = 'Car Speed Monitor (Discrete Time)'

        # Declare input/ output signals (speed, acceleration)
        spec.declare_var('speed', 'float')  # Input signal
        spec.declare_var('acc', 'float') # Accerleration
        spec.declare_var('pos', 'float')  # Position
        spec.declare_var('out', 'float')    # Output signal (formula result)
        spec.set_var_io_type('out', 'output')
        spec.set_var_io_type('speed', 'input')
        spec.set_var_io_type('acc', 'input')
        spec.set_var_io_type('pos', 'input')

        # Write STL formula
        # self.spec.spec = 'out = always[0:5](speed <= 100)' # speed is always less than 100 for the first 5 seconds
        spec.spec = (
            'out = always[0,5]('
            'speed <= 100 and '             # speed is always less than 100 for the first 30 seconds
            'acc >= -20 and acc <= 20 and '
            'pos >= 0 and pos <= 600)'
        )

        # Parse the formula to make it ready to use
        spec.parse()
        return spec
    
    def evaluate_signals(self, speed_data, acc_data, pos_data):
        
        time_data = [t for t, _ in speed_data]  # extract time list
        speed_values = [v for _, v in speed_data]
        acc_values = [v for _, v in acc_data]
        pos_values = [v for _, v in pos_data]
        
        print("time_data [0:3] samples:", time_data[:3])
        print("Speed_values [0:3] samples:", speed_values[:3])
        print("acc_values [0:3] samples:", acc_values[:3])
        print("pos_values [0:3] samples:", pos_values[:3])

        dataset = {
            'time': time_data,
            'speed': speed_values,
            'acc': acc_values,
            'pos': pos_values
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
    def init_spec(self):    
        # Define the STL specification
        spec = rtamt.StlDenseTimeSpecification()
        spec.name = 'Car Speed Monitor (Dense Time)'

        # Declare input/ output signals (speed, acceleration)
        spec.declare_var('speed', 'float')  # Input signal
        spec.declare_var('acc', 'float') # Accerleration
        spec.declare_var('pos', 'float')  # Position
        spec.declare_var('out', 'float')    # Output signal (formula result)
        spec.set_var_io_type('out', 'output')
        spec.set_var_io_type('speed', 'input')
        spec.set_var_io_type('acc', 'input')
        spec.set_var_io_type('pos', 'input')

        # Write STL formula
        # spec.spec = (
        #     'out = always[0,30]('
        #     'speed <= 100 and '             # speed is always less than 100 for the first 30 seconds
        #     'acc >= -20 and acc <= 20 and '
        #     'pos >= 0 and pos <= 600)'
        # )
        
        spec.spec = (
            'out = always(('
            '  (speed > 100 or acc < -20 or acc > 20 or pos < 0 or pos > 600)'
            ') implies eventually[0:5]('
            '  speed <= 100 and acc >= -20 and acc <= 20 and pos >= 0 and pos <= 600'
            '))'
        )

        # Parse the formula to make it ready to use
        spec.parse()
        return spec

    def evaluate_signals(self, speed_data, acc_data, pos_data):        
        # Evaluate the signal against the STL rule/ formula
        result = self.spec.evaluate(
            ['speed', speed_data],
            ['acc', acc_data],
            ['pos', pos_data]
        )
        
        # Display the robustness values and whether speed was exceeded
        print("Time\tRobustness")
        # self.get_logger().info("Time\tRobustness")
        for timestamp, robustness in result:
            status = "success" if robustness >= 0 else "violation"
            print(f"Time {timestamp}s: robustness={robustness:.2f} → {status}")
            # self.get_logger().info(f"Time {timestamp}s: robustness={robustness:.2f} → {status}")
    

# class MonitorNode(Node):
class MonitorNode():
    def __init__(self, monitor_type='discrete'): # default is discrete time
        super(MonitorNode, self).__init__('monitor_node')

        # super().__init__('Monitor_node')
        # self.get_logger().info("Monitor Node started.\n")
        # self.serial_pu = self.create_publisher(Float32MultiArray, 'command', 10)

        # Subscription to topic with speed data
        # self.subscription = self.create_subscription(
        #     Float32MultiArray,         
        #     'car_speed',
        #     self.speed_callback,
        #     10
        # )        
        # self.error_pub = self.create_publisher(String, 'error', 10)
        self.monitors = {} # TODO: map to the car id for the STLMonitor instance
        
         # Select which STL monitor to use (discrete or dense)
        if monitor_type == 'dense':
            self.monitor = DenseSTLMonitor(self.get_logger())
            print("Using Dense Time Specification")
            # self.get_logger().info("Using Dense Time Specification")
        else:
            self.monitor = DiscreteSTLMonitor(self.get_logger())
            # self.get_logger().info("Using Discrete Time Specification")
        
        speed_data, acc_data, pos_data = self.load_data_from_csv('discrete_stl_data.csv')
        
        # Feed in test signal data for speed (time, value), acceleration, and position
        self.monitor.evaluate_signals(speed_data, acc_data, pos_data)
    


def main(args=None):
    # rclpy.init(args=args)
    # global minimal_publisher
    filename = 'discrete_stl_data.csv'
    
    # Set specification type, use 'dense' for DenseSTLMonitor
    monitor_type = 'dense'
    print(f"Using {monitor_type} Time Specification")
    
    # Select which STL monitor to use (discrete or dense)
    if monitor_type == 'dense':
        monitor = DenseSTLMonitor()
    else:
        monitor = DiscreteSTLMonitor()
           
    speed_data, acc_data, pos_data = monitor.load_data_from_csv(filename)
        
    # Feed in test signal data for speed (time, value), acceleration, and position
    monitor.evaluate_signals(speed_data, acc_data, pos_data)
    
    
    # minimal_publisher = MonitorNode(monitor_type)

    # rclpy.spin(minimal_publisher)

    # minimal_publisher.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()

