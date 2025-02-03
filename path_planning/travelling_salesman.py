import osmnx as ox
import networkx as nx

# Get the road network for a city (change to your area)
place_name = "San Francisco, California, USA"
G = ox.graph_from_place(place_name, network_type="drive")

# Save the graph for future use (optional)
ox.save_graphml(G, "san_francisco.graphml")

def get_shortest_route(G, start, end):
    # Find the nearest nodes to the start & end points
    start_node = ox.distance.nearest_nodes(G, start[1], start[0])
    end_node = ox.distance.nearest_nodes(G, end[1], end[0])

    # Compute shortest path using A*
    route = nx.astar_path(G, start_node, end_node, weight="length")
    return route

import folium

def plot_route(G, route):
    # Create a folium map
    route_map = ox.plot_route_folium(G, route, route_color="blue")
    route_map.save("route.html")  # Save as interactive HTML
    return route_map

# Define start & end locations (latitude, longitude)
start_location = (37.7749, -122.4194)  # Example: San Francisco
end_location = (37.7849, -122.4094)

# Get the shortest route
route = get_shortest_route(G, start_location, end_location)

# Visualize
route_map = plot_route(G, route)



from itertools import permutations

def tsp_brute_force(G, waypoints):
    """Solves TSP by checking all permutations (only for small sets)."""
    min_route = None
    min_length = float("inf")

    for perm in permutations(waypoints):
        total_length = 0
        route = []
        
        for i in range(len(perm) - 1):
            path = nx.astar_path(G, perm[i], perm[i+1], weight="length")
            route.extend(path)
            total_length += nx.path_weight(G, path, weight="length")
        
        if total_length < min_length:
            min_length = total_length
            min_route = route

    return min_route, min_length

# Define multiple garbage collection points (nodes)
# Fix: swap lat & lon in nearest_nodes()

start_location = [37.76162287448621, -122.42659721621662]
end_location = [37.75770171114325, -122.38793067562435]
waypoints = [
    ox.distance.nearest_nodes(G, start_location[1], start_location[0]),
    ox.distance.nearest_nodes(G, end_location[1], end_location[0]),
    #ox.distance.nearest_nodes(G, -122.4294, 37.7649)  # Example extra point
]
for node in waypoints:
    if node not in G.nodes:
        print(f"Node {node} not found in graph!")


# Solve TSP for optimal garbage collection route
tsp_route, tsp_length = tsp_brute_force(G, waypoints)
plot_route(G, tsp_route)
