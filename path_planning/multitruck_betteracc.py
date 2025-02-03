import osmnx as ox
import networkx as nx
import numpy as np
import folium
from ortools.constraint_solver import routing_enums_pb2, pywrapcp

# Load the road network (San Francisco)
place_name = "San Francisco, California, USA"  # Or your city
G = ox.graph_from_place(place_name, network_type="drive")

def compute_distance_matrix(G, locations):
    num_locations = len(locations)
    distance_matrix = np.zeros((num_locations, num_locations))

    for i in range(num_locations):
        for j in range(num_locations):
            if i == j:
                continue
            try:
                distance_matrix[i][j] = nx.shortest_path_length(
                    G, locations[i], locations[j], weight="length"
                )
            except nx.NetworkXNoPath:
                print(f"No path between {i} and {j}. Setting distance to a large value.")
                distance_matrix[i][j] = 1000000  # Large value for no path

    return distance_matrix


def solve_multi_truck_vrp(distance_matrix, num_trucks):
    num_locations = len(distance_matrix)
    manager = pywrapcp.RoutingIndexManager(num_locations, num_trucks, 0)  # 0 is the depot
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
        truck_routes = []
        for truck_id in range(num_trucks):
            route = []
            index = routing.Start(truck_id)
            while not routing.IsEnd(index):
                node_index = manager.IndexToNode(index)
                route.append(int(node_index))
                index = solution.Value(routing.NextVar(index))
            truck_routes.append(route)
        return truck_routes
    return None


def plot_multi_truck_routes(G, routes, garbage_bins, colors=["red", "blue", "green", "purple"]):
    route_map = folium.Map(location=[garbage_bins[0][0], garbage_bins[0][1]], zoom_start=14)

    for truck_id, route_indices in enumerate(routes):
        truck_route_coords = []
        for i in range(len(route_indices) - 1):  # Iterate through pairs of stops
            start_bin_index = route_indices[i]
            end_bin_index = route_indices[i + 1]

            start_node = ox.distance.nearest_nodes(G, garbage_bins[start_bin_index][1], garbage_bins[start_bin_index][0])  # lon, lat
            end_node = ox.distance.nearest_nodes(G, garbage_bins[end_bin_index][1], garbage_bins[end_bin_index][0])  # lon, lat

            try:
                route = nx.shortest_path(G, start_node, end_node, weight="length")
                route_coords = [(G.nodes[node]['y'], G.nodes[node]['x']) for node in route]  # lat, lon
                truck_route_coords.extend(route_coords)
            except nx.NetworkXNoPath:
                print(f"No path between {start_bin_index} and {end_bin_index} for truck {truck_id + 1}")
                continue  # Skip to the next segment if no path is found

        if truck_route_coords:  # Check if the route is not empty
            folium.PolyLine(truck_route_coords, color=colors[truck_id % len(colors)], weight=5).add_to(route_map)

            # Add markers for garbage bins (only once per bin)
            for bin_index in route_indices:
                folium.Marker(garbage_bins[bin_index], popup=f"Truck {truck_id + 1} Stop {route_indices.index(bin_index) + 1}").add_to(route_map)

    route_map.save("multi_truck_routes.html")
    return route_map


# Garbage bin locations (Example - adjust as needed)
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
    [37.79823320727075, -122.45028679900298]
]

num_trucks = 4

bin_nodes = []
for lat, lon in garbage_bins:
    node = ox.distance.nearest_nodes(G, lon, lat)
    bin_nodes.append(node)

distance_matrix = compute_distance_matrix(G, bin_nodes)

optimal_routes_bin_indices = solve_multi_truck_vrp(distance_matrix, num_trucks)

if optimal_routes_bin_indices:
    plot_multi_truck_routes(G, optimal_routes_bin_indices, garbage_bins)

    for i, route in enumerate(optimal_routes_bin_indices):
        print(f"Truck {i + 1}: {route}")
else:
    print("No solution found for the VRP.")