#!/usr/bin/python3
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from itertools import combinations


# our goal nodes. simple x,y goal (don't care for z)
# i use 1,2,3,4 as names for new nodes so use your words pls
# or shit bouta get real confusing real fast (and prob break)
class Destination():
	def __init__(self, x, y, name):
		self.x = x
		self.y = y
		self.name = name
	# end __init__()
	def __repr__(self):
		return f"Destination({self.name}, {self.x}, {self.y})"
# end Destination()


# to have our mesh be springy, we use a force directed graph
# The thinking behind this is, given a set of points, create a
# force directed graph but when we exceed max tension (bad comms)
# we break the edge, and insert an intermediary (new drone).
# basically we run this bad boy to get where our drones should be going
def create_force_graph(destinations, max_tension, max_iter):
	
	# make graph
	initial_edges = []
	graph = nx.Graph()
	new_node_count = 0
	fixed_pos = {}
	for destination in destinations:
		graph.add_node(destination.name)
		fixed_pos[destination.name] = np.array([dest.x, dest.y])
		
	# make og edges
	core_names = [d.name for d in destinations]
	initial_edges = list(combinations(core_names, 2))
	graph.add_edges_from(initial_edges)
	
	
	# the graph moves my fixed nodes. Gonna safekeep in fixed_pos
	pop = fixed_pos.copy()
	
	for i in range(max_iter):
		print(f"\nIteration {i+1}: Nodes: {G.number_of_nodes()}, Edges: {G.number_of_edges()}")
		
		# spring sprong
		pos = nx.spring_layout(graph, pos=po, iterations=42, seed=i)
		
		# ummm thats incorrect sir
		to_break = []
		for u, v in graph.edges():
			#euclidian for now (INSERT ACTIONSERVER???)
			distance = np.linalg.norm(pos[u] - pos[v])
			if distance > max_tension:
				# GTFO
				to_break.append((u,v))
				print(f"Broke {u}<->{v} w len {distance}")
		
		# um that's actually correct sir	
		if not to_break:
			print("yippee!!")
			break
		
		# welp now we gotta break them
		for u,v in to_break:
			# chopt
			graph.remove_edge(u,v)
			
			# it took god 7 days to do this?
			new_node = f'New-{new_node_count}'
			graph.add_node(new_node)
			graph.add_edge(v, new_node)
			graph.add_edge(new_node, u)
			
			# plop brodie in the center
			pos[new_node] = (pos[u] + pos[v]) / 2.0
			
			print(f'Added node {new_node}')
			new_node_count += 1
			
	# debug visualize
	# set logic lazines to get hard vs dynamic drones
	all_nodes_set = set(graph.nodes())
	core_nodes_set = set(core_names)
	hop_nodes_final = list(all_nodes_set - core_nodes_set)
	plt.figure(figsize=(10,8))
	
	nx.draw_nodes(graph, pos, 
		nodelist=core_names, 
		node_size=1200, 
		node_color='skyblue', 
		label='Destination Nodes')
        
	nx.draw_nodes(graph, pos, 
		nodelist=hop_nodes_final, 
		node_size=400, 
		node_color='lightcoral', 
		label='Intermediary Nodes')
	nx.draw_edges(graph, pos, edge_color='gray', width=1.5)
	nx.draw_labels(graph, pos, font_size=12, font_weight='bold')
			

	plt.title(f"Dynamic Force-Directed Graph (Final State after {i+1} iterations)")
	plt.legend(scatterpoints=1)
	plt.axis('off')
	plt.show() 
    
# end create_force_graph()
if __name__ == "__main__":
	dest = []
	
	names = ["ro", "to", "po", "ko", "lo"]
	modifierx = -1
	modifiery = -1
	for i in range(5):
		dest.append(Destination(i*modifierx,i*modifiery, names[i]))
		modifierx *= -2
		modifiery *= -5.4
	create_force_graph(dest, 3.5, 50)
	
# end create_force_graph()
