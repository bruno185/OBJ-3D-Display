import sys
import os
import struct

# ============================================================================
# BSP Tree Exporter: JSON BSP -> Flat Binary for Apple IIGS
# ============================================================================
# This script reads a BSP JSON file (from build_bsp.py) and exports a flat binary
# format suitable for parsing on Apple IIGS (C or ASM).
#
# Usage:
#   python export_bsp_bin.py <bsp.json> <output.bin>
#
# Format:
#   [header]
#     uint16_t vertex_count
#     uint16_t face_count
#     uint16_t node_count
#   [vertices]
#     float32 x, float32 y, float32 z (vertex_count times)
#   [faces]
#     uint8_t vertex_per_face, uint16_t idx0, idx1, ... (face_count times)
#   [nodes]
#     uint16_t plane_face_idx
#     uint16_t faces_on_plane_count
#     uint16_t faces_on_plane_idx_start
#     int16_t front_node_idx
#     int16_t back_node_idx
#   [faces_on_plane]
#     uint16_t face_idx (for all faces_on_plane in all nodes, concat)
#
# All indices are 0-based. -1 (0xFFFF) means no node.

import json

def load_bsp_json(path):
    with open(path, 'r') as f:
        return json.load(f)

# Flatten BSP tree to a list of nodes and faces_on_plane
class FlatBSP:
    def __init__(self):
        self.nodes = []
        self.faces_on_plane = []
        self.node_indices = {}  # id(node) -> index

    def flatten(self, node):
        if node is None:
            return -1
        idx = len(self.nodes)
        self.node_indices[id(node)] = idx
        # Reserve node slot
        self.nodes.append(None)
        # Faces on plane
        faces_on = node['faces_on_plane']
        faces_on_start = len(self.faces_on_plane)
        faces_on_count = len(faces_on)
        for f in faces_on:
            self.faces_on_plane.append(f)
        # Children
        front_idx = self.flatten(node['front'])
        back_idx = self.flatten(node['back'])
        # Plane face: store as index in faces array
        plane_face = node['plane_face']
        plane_face_idx = self.find_face_idx(plane_face)
        # Store node
        self.nodes[idx] = {
            'plane_face_idx': plane_face_idx,
            'faces_on_plane_count': faces_on_count,
            'faces_on_plane_idx_start': faces_on_start,
            'front_node_idx': front_idx,
            'back_node_idx': back_idx
        }
        return idx

    def find_face_idx(self, face):
        # Faces are compared by value
        for i, f in enumerate(self.faces):
            if f == face:
                return i
        return 0xFFFF

    def run(self, bsp_tree, faces):
        self.faces = faces
        self.flatten(bsp_tree)


def write_bsp_bin(path, vertices, faces, flat_bsp):
    with open(path, 'wb') as f:
        # Header
        f.write(struct.pack('<HHH', len(vertices), len(faces), len(flat_bsp.nodes)))
        # Vertices
        for v in vertices:
            f.write(struct.pack('<fff', *v))
        # Faces
        for face in faces:
            f.write(struct.pack('<B', len(face)))
            for idx in face:
                f.write(struct.pack('<H', idx))
        # Nodes
        for node in flat_bsp.nodes:
            f.write(struct.pack('<H', node['plane_face_idx']))
            f.write(struct.pack('<H', node['faces_on_plane_count']))
            f.write(struct.pack('<H', node['faces_on_plane_idx_start']))
            f.write(struct.pack('<h', node['front_node_idx']))
            f.write(struct.pack('<h', node['back_node_idx']))
        # Faces on plane
        for face in flat_bsp.faces_on_plane:
            idx = flat_bsp.find_face_idx(face)
            f.write(struct.pack('<H', idx))

def main():
    if len(sys.argv) < 3:
        print("Usage: python export_bsp_bin.py <bsp.json> <output.bin>")
        sys.exit(1)
    json_path = sys.argv[1]
    bin_path = sys.argv[2]
    data = load_bsp_json(json_path)
    vertices = data['vertices']
    faces = data['faces']
    bsp_tree = data['bsp_tree']
    flat_bsp = FlatBSP()
    flat_bsp.run(bsp_tree, faces)
    write_bsp_bin(bin_path, vertices, faces, flat_bsp)
    print(f"Exported BSP binary to {bin_path}")

if __name__ == '__main__':
    main()
