######################################################################
#                                                                    
# Part of SILLEO-SCNS, routing abstraction for LEO satellite networks
# Copyright (C) 2020  Benjamin S. Kempton
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
######################################################################

import networkx as nx
from abc import ABC, abstractmethod
from typing import List, Optional, Union
import math


class LEORouter(ABC):
    """
    Abstract base class for LEO satellite network routing algorithms.
    
    This class defines the interface that all routing algorithms must implement
    for the SILLEO-SCNS simulator. All routing algorithms work with NetworkX
    graphs and must provide a find_route method with the same signature.
    
    Attributes
    ----------
    name : str
        The name of the routing algorithm
    description : str
        A description of the routing algorithm
    """
    
    def __init__(self, name: str, description: str):
        """
        Initialize the router with a name and description.
        
        Parameters
        ----------
        name : str
            The name of the routing algorithm
        description : str
            A description of the routing algorithm
        """
        self.name = name
        self.description = description
    
    @abstractmethod
    def find_route(self, 
                   graph: nx.Graph, 
                   source: Union[str, int], 
                   target: Union[str, int], 
                   weight: str = 'distance') -> Optional[List[Union[str, int]]]:
        """
        Find the optimal route between two nodes in the network.
        
        This is the main method that all routing algorithms must implement.
        It should return the optimal path between source and target nodes
        according to the specific routing algorithm.
        
        Parameters
        ----------
        graph : nx.Graph
            The NetworkX graph representing the satellite network
        source : Union[str, int]
            The source node identifier
        target : Union[str, int]
            The target node identifier
        weight : str, optional
            The edge attribute to use as weight for path calculation (default: 'distance')
            
        Returns
        -------
        Optional[List[Union[str, int]]]
            A list of nodes representing the optimal path from source to target,
            or None if no path exists
        """
        pass
    
    def __str__(self) -> str:
        """Return a string representation of the router."""
        return f"{self.name}: {self.description}"


class OSPFRouter(LEORouter):
    """
    OSPF (Open Shortest Path First) routing algorithm implementation.

    For the purposes of this simulation, which operates on a known graph at each
    timestep, the pathfinding logic of OSPF is equivalent to using Dijkstra's
    algorithm, which is provided by NetworkX. This class abstracts that logic
    under the OSPF name.
    """

    def __init__(self):
        """Initialize the OSPF router."""
        super().__init__(
            name="OSPF",
            description="OSPF (Dijkstra's) shortest path algorithm using NetworkX"
        )

    def find_route(self,
                   graph: nx.Graph,
                   source: Union[str, int],
                   target: Union[str, int],
                   weight: str = 'distance') -> Optional[List[Union[str, int]]]:
        """
        Find the shortest path using OSPF's underlying Dijkstra's algorithm.

        Parameters
        ----------
        graph : nx.Graph
            The NetworkX graph representing the satellite network
        source : Union[str, int]
            The source node identifier
        target : Union[str, int]
            The target node identifier
        weight : str, optional
            The edge attribute to use as cost/weight for path calculation (default: 'distance')

        Returns
        -------
        Optional[List[Union[str, int]]]
            A list of nodes representing the shortest path from source to target,
            or None if no path exists

        Raises
        ------
        nx.exception.NetworkXNoPath
            If no path exists between source and target
        nx.exception.NodeNotFound
            If source or target node is not in the graph
        """
        try:
            # OSPF uses Dijkstra's algorithm to find the shortest path
            path = nx.shortest_path(
                graph,
                source=str(source),
                target=str(target),
                weight=weight
            )
            return path
        except (nx.exception.NetworkXNoPath, nx.exception.NodeNotFound):
            # Re-raise the exception to be handled by the simulation
            raise


class AODVRouter(LEORouter):
    """
    AODV (Ad-hoc On-demand Distance Vector) routing algorithm implementation.

    This implementation simulates AODV's on-demand route discovery process
    by simulating RREQ flooding: each node only knows its direct neighbors
    and forwards the RREQ. The route is discovered as the RREQ propagates,
    not by global BFS. This is less efficient than OSPF and more realistic
    for a reactive protocol.
    
    In this version, the RREQ floods the entire network, collecting all possible
    paths to the destination, and the shortest one is selected. This simulates
    the flooding behavior of AODV route discovery.
    
    The implementation uses:
    - A TTL (hop limit) to avoid infinite loops
    - Simple cycle prevention to avoid revisiting nodes in the same path
    - Collection of all possible paths to find the shortest one
    """

    def __init__(self):
        """Initialize the AODV router."""
        super().__init__(
            name="AODV",
            description="AODV (on-demand, hop count) with full RREQ flooding"
        )

    def find_route(self,
                   graph: nx.Graph,
                   source: Union[str, int],
                   target: Union[str, int],
                   weight: str = None) -> Optional[List[Union[str, int]]]:
        """
        Simulate AODV's reactive RREQ flooding to discover a route.
        Flood the network and return the first path found to the destination.

        A TTL (hop limit) is used to avoid infinite loops. This simulates
        what a real AODV implementation would do in a distributed environment.

        TTL is set to a fixed value (default: 100) to limit flooding and avoid excessive computation.
        A computation budget (MAX_ITERATIONS) is also enforced to prevent the simulation from freezing
        due to excessive computation. If the budget is exceeded, the algorithm aborts and raises NetworkXNoPath.

        Parameters
        ----------
        graph : nx.Graph
            The NetworkX graph representing the satellite network.
        source : Union[str, int]
            The source node identifier.
        target : Union[str, int]
            The target node identifier.
        weight : str, optional
            This parameter is ignored, as AODV uses hop count.

        Returns
        -------
        Optional[List[Union[str, int]]]
            A list of nodes representing the first path found in hops.

        Raises
        ------
        nx.exception.NodeNotFound
            If the source or target node is not in the graph.
        nx.exception.NetworkXNoPath
            If no path exists between the source and target.
        """
        source, target = str(source), str(target)

        if source not in graph:
            raise nx.NodeNotFound(f"Source node {source} not in graph")
        if target not in graph:
            raise nx.NodeNotFound(f"Target node {target} not in graph")

        FIXED_TTL = 100  # Reasonable TTL for RREQs to limit flooding
        MAX_ITERATIONS = 1000  # Computation budget to keep simulation responsive
        queue = [(source, [source], FIXED_TTL)]  # (current_node, path_so_far, ttl)
        iterations = 0

        while queue:
            iterations += 1
            if iterations > MAX_ITERATIONS:
                # Too much computation, meaning that the algorithm took too long to compute a route, thus no route was found. Recall, satellites are moving fast, so the pathfinding algorithm should be fast.
                raise nx.NetworkXNoPath(f"Computation budget exceeded for AODV between {source} and {target}")

            current_node, path, ttl = queue.pop(0)
            if ttl <= 0:
                continue

            if current_node == target:
                # Return the first path found (AODV typically returns the first route discovered)
                return path

            for neighbor in graph.neighbors(current_node):
                if neighbor not in path:  # Prevent cycles in a single path
                    queue.append((neighbor, path + [neighbor], ttl - 1))

        # No path found
        raise nx.NetworkXNoPath(f"No path found between {source} and {target}")


class GPSRRouter(LEORouter):
    """
    GPSR (Greedy Perimeter Stateless Routing) implementation.

    This router uses geographic positioning to forward packets. It operates in
    two modes:
    1.  Greedy Mode: Forwards the packet to the neighbor geographically
        closest to the destination.
    2.  Perimeter Mode: If a packet reaches a node that is closer than all
        its neighbors (a local maximum), it switches to traversing the
        perimeter of the region using a right-hand rule.
    """

    def __init__(self):
        """Initialize the GPSR router."""
        super().__init__(
            name="GPSR",
            description="Greedy Perimeter Stateless Routing using node coordinates"
        )

    def _distance(self, pos1, pos2):
        """Calculates the Euclidean distance between two points in 3D space."""
        return ((pos1['x'] - pos2['x'])**2 + (pos1['y'] - pos2['y'])**2 + (pos1['z'] - pos2['z'])**2)**0.5

    def _sort_neighbors_by_angle(self, graph, current_id, prev_id):
        """
        Sorts neighbors of a node counter-clockwise.

        This is used for the right-hand rule in perimeter mode. The angle is
        calculated in the XY-plane relative to the vector from the previous
        node to the current one.
        """
        current_pos = graph.nodes[current_id]
        prev_pos = graph.nodes[prev_id]

        # Vector from previous to current node
        ref_vector = (current_pos['x'] - prev_pos['x'], current_pos['y'] - prev_pos['y'])
        ref_angle = math.atan2(ref_vector[1], ref_vector[0])

        angles = []
        for neighbor_id in graph.neighbors(current_id):
            if neighbor_id == prev_id:
                continue
            
            neighbor_pos = graph.nodes[neighbor_id]
            # Vector from current to neighbor
            vec = (neighbor_pos['x'] - current_pos['x'], neighbor_pos['y'] - current_pos['y'])
            angle = math.atan2(vec[1], vec[0])
            
            # Get angle difference and normalize to [0, 2*pi]
            angle_diff = angle - ref_angle
            if angle_diff < 0:
                angle_diff += 2 * math.pi
                
            angles.append((angle_diff, neighbor_id))
            
        angles.sort()
        # Return a list of neighbor IDs sorted CCW
        return [neighbor_id for _, neighbor_id in angles]

    def find_route(self,
                   graph: nx.Graph,
                   source: Union[str, int],
                   target: Union[str, int],
                   weight: str = None) -> Optional[List[Union[str, int]]]:
        """
        Finds a route using the GPSR algorithm.
        """
        import math # Import math here to ensure it's available
        
        source, target = str(source), str(target)

        if source not in graph:
            raise nx.NodeNotFound(f"Source node {source} not in graph")
        if target not in graph:
            raise nx.NodeNotFound(f"Target node {target} not in graph")

        path = [source]
        current_id = source
        mode = "greedy"
        perimeter_entry_pos = None

        max_hops = graph.number_of_nodes() * 2  # Safety break for loops
        
        while current_id != target:
            if len(path) > max_hops:
                raise nx.NetworkXNoPath("GPSR path exceeded max hops, likely a loop.")
            
            current_pos = graph.nodes[current_id]
            target_pos = graph.nodes[target]
            neighbors = list(graph.neighbors(current_id))

            if not neighbors:
                raise nx.NetworkXNoPath(f"Node {current_id} is disconnected.")

            # If in perimeter mode, check if we can switch back to greedy
            if mode == "perimeter":
                if self._distance(current_pos, target_pos) < self._distance(perimeter_entry_pos, target_pos):
                    mode = "greedy"
            
            # --- GREEDY MODE ---
            if mode == "greedy":
                min_dist_to_target = self._distance(current_pos, target_pos)
                closest_neighbor_id = None
                for neighbor_id in neighbors:
                    dist = self._distance(graph.nodes[neighbor_id], target_pos)
                    if dist < min_dist_to_target:
                        min_dist_to_target = dist
                        closest_neighbor_id = neighbor_id
                
                if closest_neighbor_id is not None:
                    current_id = closest_neighbor_id
                    path.append(current_id)
                    continue
                else:
                    # Stuck, switch to perimeter mode
                    mode = "perimeter"
                    perimeter_entry_pos = current_pos
                    # Fallthrough to execute perimeter mode in the same step

            # --- PERIMETER MODE ---
            if mode == "perimeter":
                # Use right-hand rule to find next hop
                prev_id = path[-2] if len(path) > 1 else source
                sorted_neighbors = self._sort_neighbors_by_angle(graph, current_id, prev_id)
                
                if not sorted_neighbors:
                    raise nx.NetworkXNoPath(f"Stuck in perimeter mode at {current_id}, no other neighbors.")

                # Pick the first neighbor counter-clockwise from the incoming edge
                next_hop_id = sorted_neighbors[0]
                
                current_id = next_hop_id
                path.append(current_id)

        return path
