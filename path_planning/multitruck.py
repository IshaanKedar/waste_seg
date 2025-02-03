import osmnx as ox
import networkx as nx
import numpy as np
import folium
from ortools.constraint_solver import routing_enums_pb2, pywrapcp

# Load the road network
place_name = "San Francisco, California, USA"  # Or your city
G = ox.graph_from_place(place_name, network_type="drive")

# Save the graph (optional)
# ox.save_graphml(G, "san_francisco.graphml")


def get_shortest_route(G, start, end):
    start_node = ox.distance.nearest_nodes(G, start[1], start[0])
    end_node = ox.distance.nearest_nodes(G, end[1], end[0])
    return nx.astar_path(G, start_node, end_node, weight="length")


def plot_route(G, route, color="blue"):
    return ox.plot_route_folium(G, route, route_color=color)


def compute_distance_matrix(G, locations):
    num_locations = len(locations)
    distance_matrix = np.zeros((num_locations, num_locations))

    for i in range(num_locations):
        for j in range(num_locations):
            if i == j:
                continue
            distance_matrix[i][j] = nx.shortest_path_length(
                G, locations[i], locations[j], weight="length"
            )

    return distance_matrix


def solve_multi_truck_vrp(distance_matrix, num_trucks):
    num_locations = len(distance_matrix)
    manager = pywrapcp.RoutingIndexManager(num_locations, num_trucks, 0)
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
        route_coords = [garbage_bins[i] for i in route_indices]
        folium.PolyLine(route_coords, color=colors[truck_id % len(colors)], weight=5).add_to(route_map)

        for i, loc in enumerate(route_coords):
            folium.Marker(loc, popup=f"Truck {truck_id+1} Stop {i+1}").add_to(route_map)

    route_map.save("multi_truck_routes.html")
    return route_map


# Garbage bin locations
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

# Convert lat/lon to nearest nodes and store indices
bin_nodes = []
bin_node_indices = []
for i, (lat, lon) in enumerate(garbage_bins):
    node = ox.distance.nearest_nodes(G, lon, lat)
    bin_nodes.append(node)
    bin_node_indices.append(i)

distance_matrix = compute_distance_matrix(G, bin_nodes)

optimal_routes_node_indices = solve_multi_truck_vrp(distance_matrix, num_trucks)

optimal_routes_bin_indices = []
if optimal_routes_node_indices:
    for route_node_indices in optimal_routes_node_indices:
        route_bin_indices = [bin_node_indices[node_index] for node_index in route_node_indices]
        optimal_routes_bin_indices.append(route_bin_indices)
else:
    print("No solution found for the VRP.")
    exit()

plot_multi_truck_routes(G, optimal_routes_bin_indices, garbage_bins)

# Example: Print the routes (optional)
for i, route in enumerate(optimal_routes_bin_indices):
    print(f"Truck {i+1}: {route}")