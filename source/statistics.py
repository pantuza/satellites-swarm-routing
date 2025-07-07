######################################################################
#                                                                    
# Part of SILLEO-SCNS, statistics logging for routing protocols
# Copyright (C) 2024  Benjamin S. Kempton & Pantelis P.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
######################################################################

import pandas as pd
import time
import os
from pathlib import Path

class StatisticsLogger:
    """
    A logger for collecting and saving routing protocol statistics.

    This class creates a Parquet file immediately and appends data in real-time.
    """

    def __init__(self, routing_name, num_nodes, network_design, orbit_height, source_node=None, dest_node=None):
        """
        Initializes the logger for a specific simulation configuration.

        Args:
            routing_name (str): The name of the routing protocol.
            num_nodes (int): The total number of nodes in the network graph.
            network_design (str): The network design/linking method.
            orbit_height (float): The orbital height of the satellites.
            source_node (str, optional): The source node name for the route.
            dest_node (str, optional): The destination node name for the route.
        """
        self.routing_name = routing_name
        self.num_nodes = num_nodes
        self.network_design = network_design
        self.orbit_height = orbit_height
        self.source_node = source_node
        self.dest_node = dest_node
        
        # Create the directory if it doesn't exist
        self.output_dir = Path(__file__).parent / 'data'
        self.output_dir.mkdir(exist_ok=True)
        
        # Construct the filename with more descriptive information
        timestamp = int(time.time())
        
        # Create a safe filename with network design and node pair
        network_safe = network_design.replace('+', 'plus').replace('-', 'minus')
        
        if source_node and dest_node:
            # Create safe node names for filename (replace spaces and special chars)
            source_safe = source_node.replace(' ', '_').replace('-', '_').replace('.', '_')
            dest_safe = dest_node.replace(' ', '_').replace('-', '_').replace('.', '_')
            node_pair = f"{source_safe}-{dest_safe}"
        else:
            node_pair = "general"
        
        filename = f"{self.routing_name}-{network_safe}-{self.num_nodes}nodes-{node_pair}-{timestamp}.parquet"
        self.filepath = self.output_dir / filename
        
        # Create an empty DataFrame with the correct schema and save it immediately
        self.df = pd.DataFrame(columns=[
            'protocol_name', 'number_of_nodes', 'orbit_height', 'network_design',
            'timestamp', 'source_node', 'destination_node', 'time_taken_us',
            'num_hops', 'total_distance', 'route_path'
        ])
        
        # Save the empty file to create it
        self.df.to_parquet(self.filepath, engine='pyarrow')
        print(f"[StatisticsLogger] Created file: {self.filepath}")
        
        self.record_count = 0

    def log_route(self, time_taken_us, num_hops, total_distance, source_node, dest_node, path, timestamp):
        """
        Logs a single routing event and immediately appends it to the file.
        """
        entry = {
            'protocol_name': self.routing_name,
            'number_of_nodes': self.num_nodes,
            'orbit_height': self.orbit_height,
            'network_design': self.network_design,
            'timestamp': timestamp,
            'source_node': source_node,
            'destination_node': dest_node,
            'time_taken_us': time_taken_us,
            'num_hops': num_hops,
            'total_distance': total_distance,
            'route_path': str(path),  # Store path as string for Parquet compatibility
        }
        
        # Append the new record to the DataFrame
        self.df = pd.concat([self.df, pd.DataFrame([entry])], ignore_index=True)
        
        # Save the updated DataFrame to the file
        self.df.to_parquet(self.filepath, engine='pyarrow')
        
        self.record_count += 1
        print(f"[StatisticsLogger] Added record #{self.record_count}: {source_node} -> {dest_node} ({num_hops} hops, {time_taken_us:.2f}μs)")

    def get_record_count(self):
        """Returns the number of records logged so far."""
        return self.record_count 