import osmnx as ox
import networkx as nx
import numpy as np
import folium
from ortools.constraint_solver import routing_enums_pb2, pywrapcp
import random

# Load the road network (San Francisco)
place_name = "San Francisco, California, USA"
G = ox.graph_from_place(place_name, network_type="drive")

def compute_distance_matrix(G, nodes):
    num_locations = len(nodes)
    distance_matrix = np.zeros((num_locations, num_locations))
    for i in range(num_locations):
        for j in range(num_locations):
            if i == j:
                continue
            try:
                distance_matrix[i][j] = nx.shortest_path_length(
                    G, nodes[i], nodes[j], weight="length"
                )
            except nx.NetworkXNoPath:
                print(f"No path between {i} and {j}. Setting distance to a large value.")
                distance_matrix[i][j] = 1000000  # Large value for no path
    return distance_matrix

def solve_multi_truck_vrp_combined(distance_matrix, num_trucks):
    total_nodes = len(distance_matrix)
    depot_indices = list(range(num_trucks))  # Depots are at indices 0,1,...,num_trucks-1
    manager = pywrapcp.RoutingIndexManager(total_nodes, num_trucks, depot_indices, depot_indices)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(distance_matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        routes = []
        for truck_id in range(num_trucks):
            index = routing.Start(truck_id)
            route = []
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                route.append(node_index)
                index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(index))  # add the end node (depot)
            routes.append(route)
        return routes
    else:
        return None
    
# to save each map separately
# def plot_multi_truck_routes(G, routes, start_nodes, bin_nodes, num_trucks, colors=["red", "blue", "green", "purple"]):
#     for truck_id, route in enumerate(routes):
#         route_map = folium.Map(location=(G.nodes[start_nodes[0]]['y'], G.nodes[start_nodes[0]]['x']), zoom_start=14)  # New map for each truck
#         truck_route_coords = []
#         for node_index_in_combined in route:
#             actual_node_id = combined_nodes[node_index_in_combined]
#             coord = (G.nodes[actual_node_id]['y'], G.nodes[actual_node_id]['x'])
#             truck_route_coords.append(coord)

#         if truck_route_coords:
#             folium.PolyLine(truck_route_coords, color=colors[truck_id % len(colors)], weight=5).add_to(route_map)
#             for stop_num, coord in enumerate(truck_route_coords):
#                 popup_text = f"Truck {truck_id+1} Stop {stop_num+1}"
#                 if node_index_in_combined < num_trucks:
#                     popup_text += " (Depot)"
#                 folium.Marker(coord, popup=popup_text).add_to(route_map)

#         route_map.save(f"multi_truck_routes_truck_{truck_id + 1}.html")  # Save each map separately


# to have a combined map 

def plot_multi_truck_routes(G, routes, start_nodes, bin_nodes, num_trucks, colors=["red", "blue", "green", "purple"]):
    route_map = folium.Map(location=(G.nodes[start_nodes[0]]['y'], G.nodes[start_nodes[0]]['x']), zoom_start=14)

    for truck_id, route in enumerate(routes):
        truck_route_coords = []
        for node_index_in_combined in route:
            actual_node_id = combined_nodes[node_index_in_combined]
            coord = (G.nodes[actual_node_id]['y'], G.nodes[actual_node_id]['x'])
            truck_route_coords.append(coord)

        if truck_route_coords:
            # Use different colors for each truck
            folium.PolyLine(truck_route_coords, color=colors[truck_id % len(colors)], weight=5).add_to(route_map)  
            for stop_num, coord in enumerate(truck_route_coords):
                popup_text = f"Truck {truck_id+1} Stop {stop_num+1}"
                if node_index_in_combined < num_trucks:
                    popup_text += " (Depot)"
                folium.Marker(coord, popup=popup_text).add_to(route_map)

    route_map.save("multi_truck_routes_all_trucks.html")  # Save combined map

# ----- Setup Data -----

garbage_bins = [
    [37.753192625426536, -122.42978020660529],
    [37.756235125413994, -122.4323862563986],
    [37.75556419942164, -122.43453763236622],
    [37.761242249228914, -122.43075019189047],
    [37.76193694662096, -122.4219923533925],
    [37.760296432045294, -122.41845310191188],
    [37.75621500206264, -122.38773268398667],
    [37.77304514024724, -122.46309844177537],
    [37.79611810369347, -122.44291183057653],
    [37.79823320727075, -122.45028679900298],
    
]

num_trucks = 4

start_locations = [[37.37570862929876, -122.06207104982914],[37.41744980769629, -121.97447379120439],[37.542263939402666, -122.36773819817923],[37.42161626121509, -121.88740079040981]]


start_nodes = []
for lat, lon in start_locations:
    node = ox.distance.nearest_nodes(G, lon, lat)
    start_nodes.append(node)

bin_nodes = []
for lat, lon in garbage_bins:
    node = ox.distance.nearest_nodes(G, lon, lat)
    bin_nodes.append(node)

combined_nodes = start_nodes + bin_nodes
combined_distance_matrix = compute_distance_matrix(G, combined_nodes)

routes = solve_multi_truck_vrp_combined(combined_distance_matrix, num_trucks)

if routes:
    plot_multi_truck_routes(G, routes, start_nodes, bin_nodes, num_trucks)
    for i, route in enumerate(routes):
        print(f"Truck {i+1} route (node indices in combined list): {route}")
else:
    print("No solution found for the VRP.")