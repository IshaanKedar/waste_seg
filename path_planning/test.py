import osmnx as ox
import networkx as nx

# Load road network for a city
place_name = "San Francisco, California, USA"
G = ox.graph_from_place(place_name, network_type="drive")

import googlemaps

import requests

HERE_API_KEY = "wxknDlMudbkEb7mX5lVviMyA-dK9A2wF02APm5spgfc"

def get_here_traffic_data(latitude, longitude):
    """Fetch real-time traffic data from HERE Maps API."""
    url = f"https://traffic.ls.hereapi.com/traffic/6.3/flow.json?apiKey={HERE_API_KEY}&prox={latitude},{longitude},5000"
    
    response = requests.get(url)
    if response.status_code == 200:
        traffic_data = response.json()
        return traffic_data
    else:
        print("Error fetching traffic data:", response.status_code)
        return None

# Example: Fetch traffic near San Francisco
traffic_info = get_here_traffic_data(37.7749, -122.4194)
print("Real-time traffic data:", traffic_info)


# Define garbage collection points (latitude, longitude)
collection_points = [
    (37.7749, -122.4194),  # Point A
    (37.7849, -122.4094),  # Point B
    (37.7649, -122.4294),  # Point C
    (37.7549, -122.4194),  # Point D
    (37.7449, -122.4094),  # Point E
]

# Define the garbage truck depot (starting point)
depot = (37.7749, -122.4194)

# Number of garbage trucks available
num_trucks = 2

import numpy as np

def compute_distance_matrix(locations):
    """Compute a distance matrix considering real-time traffic."""
    size = len(locations)
    matrix = np.zeros((size, size))

    for i in range(size):
        for j in range(size):
            if i == j:
                continue  # Skip self-to-self

            # Get travel time with traffic
            travel_time = get_real_time_traffic(locations[i], locations[j])
            if travel_time is not None:
                matrix[i][j] = travel_time  # Time in seconds
            else:
                matrix[i][j] = 10000  # Large value if no data (avoid this route)

    return matrix

# Create a distance matrix for the depot + collection points
locations = [depot] + collection_points
distance_matrix = compute_distance_matrix(locations)


from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def solve_vrp(distance_matrix, num_trucks):
    """Solves the multi-truck routing problem using OR-Tools."""
    size = len(distance_matrix)
    
    # Create routing index manager
    manager = pywrapcp.RoutingIndexManager(size, num_trucks, 0)  # Depot index = 0

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Define cost of each arc (edge)
    def distance_callback(from_index, to_index):
        """Returns the travel time between nodes."""
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(distance_matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Define search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    # Extract routes
    if solution:
        routes = []
        for vehicle_id in range(num_trucks):
            route = []
            index = routing.Start(vehicle_id)
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                route.append(node_index)
                index = solution.Value(routing.NextVar(index))
            routes.append(route)
        return routes
    else:
        return None

# Solve VRP and get optimal truck routes
optimal_routes = solve_vrp(distance_matrix, num_trucks)
print("Optimized truck routes:", optimal_routes)


import folium

def plot_vrp_routes(routes, locations):
    """Visualizes multi-truck garbage collection routes on a map."""
    map_center = locations[0]  # Depot
    route_map = folium.Map(location=map_center, zoom_start=14)

    colors = ["red", "blue", "green", "purple", "orange"]  # Different colors for each truck

    for truck_id, route in enumerate(routes):
        route_coords = [locations[i] for i in route]
        folium.PolyLine(route_coords, color=colors[truck_id % len(colors)], weight=5).add_to(route_map)

        # Add markers
        for i, loc in enumerate(route_coords):
            folium.Marker(loc, popup=f"Truck {truck_id + 1} Stop {i+1}").add_to(route_map)

    route_map.save("multi_truck_routes.html")  # Save as interactive HTML
    return route_map

# Plot the optimized multi-truck routes
plot_vrp_routes(optimal_routes, locations)
