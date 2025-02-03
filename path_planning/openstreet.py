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
