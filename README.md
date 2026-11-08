# Drone_planner

The objective of the Drone_planner scheme is to coordinate a set of heterogeneous UAS for the optimal fulfillment of two types of missions: visiting several points of interest and searching a given area of ​​interest. The result of the scheme is a set of safe and feasible routes (one for each UAS) that optimally solve the visiting or searching missions. In addition, there may be NFZs in the flight area that the UAS must avoid. The scheme is decomposed into two main layers: 1) route generation in which the vehicle routing problem (VRP) is solved using OR-Tools and 2) route planning that determines a smooth and safe route for each UAS to fly its route while avoiding the NFZs using A*

## 📋 Features

- [x] **Multi-depot**: Define several departure and arrival depots. Each UAV can take off and land at different locations.

- [x] **Heterogeneity**: Each vehicle can possess distinct capabilities, including various velocities and battery requirements (maximum time of flight).

- [x] **HMI**: Interact with the web application to define the problem interactively. Add addresses or coordinates to visit, and specify each vehicle's capabilities. Generate and save the routes and the problem definition.

- [x] [**OR-Tools**](https://developers.google.com/optimization): Currently leveraging the [OR-Tools open-source library for combinatorial optimization](https://github.com/google/or-tools) to compute the routes.


## ⚙️ Installation

1. To install it you can create a virtual environment (optional).

    python -m venv venv
    venv\Scripts\activate

2. Go to the path.

    cd Path\to\Drone_planner


3. Install the requirements.

    pip install -r requirements.txt


## 🖥️ Usage

To use the HMI, you need a [Google Maps API Key](https://developers.google.com/maps/documentation/javascript/get-api-key) to load the map into the webpage. 

1. Set your Google Maps API Key as an enviroment variable.    
    
    setx MAPS_API_KEY <YOUR_API_KEY>

2. Befor running the application:
    
    3.1. Modify the path to DEM in the following places:

        - Path\to\Drone_planner\p_generator.py: dem_path = path\to\your\DEM
        - Path\to\Drone_planner\app\routes\routes.py: dem_path = path\to\your\DEM
        - Path\to\Drone_planner\app\routes\loaders.py: dem_path = path\to\your\DEM

    3.2. Modify the path to the flight plans:

        - Path\to\Drone_planner\app\routes\routes.py: fp_dir = path\to\your\FlightPlans

3. Launch the web app.

    python3 main.py


## ✍🏼 Problem Definition

You can define the problem by placeing two input files mission.yaml and uas.yaml into the following path:
Path\to\Drone_planner\cfg\pd

Then by pressing "Load Problem Definition from File" button in the HMI, the problem components will be generated and visualizes 
on the screen

## 🗺️ Routes Computation

By pushing "Generate Routes" button, the routes will be computed.

We support [OR-Tools open-source Python library](https://github.com/google/or-tools) for routes calculation. It is [well-documented](https://developers.google.com/optimization/reference/python/index_python) and provides [useful utilities for routing problems](https://developers.google.com/optimization/reference/python/constraint_solver/pywrapcp), such as the VRP. Refer to their [guide](https://developers.google.com/optimization/routing) for examples related to the VRP problem.

In this project, we define the `find_routes` function in the **/path/to/Drone_planner/app/utils/or_tools_method.py** file to compute optimized routes. Then We use `adapt_solution` function to convert the OR-Tools format to the required format.  This function should take the output of the `find_routes` function and produce a dictionary in the following format:
```python
result = {
    'routes': {
        # N vehicle IDs, should be integers from 0 to (N-1)
        0: {
             # Latitude and longitude list of nodes to visit
            'coordinates': [(37.3830519, -5.9902257), (37.3842311, -5.9709563), ...],
             # A list representing the indexes of the coordinates of the routes in the problem definition
            'nodes': [20, 10, 2, ...],
             # Estimated time to complete the route [s]
            'time': 12069,
            # Hexadecimal color to visualize the route path
            'color': '#00FF00',
            # Velocity of the vehicle [m/s]
            'velocity': 3.2, 
        }
        1: ...
    },
    # Time to complete the longest route [s]
    'total_time': 15000
}
```


