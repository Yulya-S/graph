import matplotlib.pyplot as plt
import networkx as nx
from networkx.drawing.nx_pydot import graphviz_layout
import json
import shapely
import math


def get_spanning_tree(graph):
    edges, nodes = [], []
    for node in graph.nodes:
        nodes.append(node)
        connected_edges = filter(lambda edge: edge not in nodes,
                                 graph.edges(node))
        min_edge = min(connected_edges,
                       key=lambda e: graph.get_edge_data(*e)["weight"])
        edges.append(min_edge +
                     (graph.get_edge_data(*min_edge)["weight"],))
    result = nx.Graph()
    #print(edges)
    result.add_weighted_edges_from(edges)
    result.add_nodes_from(nodes)
    return result


def get_clusters(spanning_tree: nx.Graph, n_clusters: int):
    """Splits the tree into n_clusters clusters"""
    # Sort edges of the spanning_tree
    sorted_edges = sorted(spanning_tree.edges(data=True), key=lambda x: x[2]['weight'], reverse=True)
    copy_tree = spanning_tree.copy()
    for i in range(n_clusters - 1):
        if len(sorted_edges) > i:
            copy_tree.remove_edge(*sorted_edges[i][:2])
    draw_graph(copy_tree)
    return list(nx.connected_components(copy_tree))


def draw_graph(graph):
    pos = nx.spring_layout(graph)
    plt.figure(figsize=(7, 6))
    nx.draw_networkx_nodes(graph, pos)
    nx.draw_networkx_edges(graph, pos, graph.edges)
    #nx.draw_networkx_labels(graph, pos, font_size=20)
    plt.show()


def distance(site1, site2):
    loc1 = site1["location"]
    loc2 = site2["location"]
    r = 6371
    phi1 = math.radians(loc1["lat"])
    phi2 = math.radians(loc2["lat"])
    dphi = math.radians(loc2["lat"] - loc1["lat"])
    dlambda = math.radians(loc2["lon"] - loc1["lon"])
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    c = 2 * math.atan2(a ** 0.5, (1 - a) ** 0.5)
    return r * c


def make_graph(sites, max_distance=400):
    edges = []
    nodes = []
    for i, site in enumerate(sites[:-1]):
        # print(f"{i+1}/{len(sites)-1}")
        nodes.append(site["code"])
        min_distance = 10 ** 16
        min_dest = None
        min_dest_flag = True
        for dest in sites[i + 1:]:
            d = distance(site, dest)
            if d <= max_distance:
                edges.append((site["code"], dest["code"], d))
                min_dest_flag = False
            if d < min_distance:
                min_dest = dest
                min_distance = d
        if min_dest_flag:
            edges.append((site["code"], dest["code"], min_distance))
    return edges, nodes


with open("base.json", "r") as f:
    sites = json.load(f)
    # print(len(sites))
    lons = []
    lats = []
    for site in sites:
        lons.append(site["location"]["lon"])
        lats.append(site["location"]["lat"])

with open("continents.json", "r") as f:
    continents = json.load(f)
    north_america_polygon = continents["features"][1]["geometry"]["coordinates"]
# print(north_america_polygon[0][0])

max_polygon = []
max_area = 0

for i in range(len(north_america_polygon)):
    points = north_america_polygon[i][0]
    area = shapely.Polygon(points).area
    if area > max_area:
        max_polygon = points
        max_area = area

xs, ys = zip(*max_polygon)

north_america = shapely.Polygon(max_polygon)
north_america_sites = []
for site in sites:
    p = shapely.Point((site["location"]["lon"]), (site["location"]["lat"]))
    if north_america.contains(p):
        north_america_sites.append(site)

flons = []
flats = []
for site in north_america_sites:
    flons.append(site["location"]["lon"])
    flats.append(site["location"]["lat"])
# print(len(north_america_sites))


plt.scatter(lons, lats)
plt.scatter(flons, flats)
plt.plot(xs, ys)
#plt.show()

# print(distance(north_america_sites[1], north_america_sites[3]))
edges, nodes = make_graph(north_america_sites)
graph = nx.Graph()
# graph.add_edges_from(edges)
graph.add_nodes_from(nodes)
graph.add_weighted_edges_from(edges)

cluster = get_clusters(nx.minimum_spanning_tree(graph), 40)
print(cluster)
print(len(cluster))
