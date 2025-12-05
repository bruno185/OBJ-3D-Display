#!/usr/bin/env python3
"""
============================================================================
OBJ to BSP Binary Converter for Apple IIGS
============================================================================
This script performs the complete pipeline:
  1. Filter OBJ file (keep only vertices and faces)
  2. Center and scale the model to fit Fixed32 limits
  3. Build BSP tree from faces
  4. Export to binary format for Apple IIGS
  5. Optionally deploy to Apple disk image via Cadius

Usage:
    python obj_to_bsp.py <input.obj> <output.bin> [target_size] [-d]

Arguments:
    input.obj:    Path to input OBJ file (required)
    output.bin:   Path to output binary BSP file (required)
    target_size:  Scaling target size (optional, default: 10.0)
    -d, --deploy: Deploy to Apple disk image via Cadius

Example:
    python obj_to_bsp.py cone.obj cone.bsp 10
    python obj_to_bsp.py cone.obj cone.bsp -d
    python obj_to_bsp.py cone.obj cone.bsp 10 -d

Binary Format:
    [header]
      uint16_t vertex_count
      uint16_t face_count
      uint16_t node_count
    [vertices]
      float32 x, float32 y, float32 z (vertex_count times)
    [faces]
      uint8_t vertex_per_face, uint16_t idx0, idx1, ... (face_count times)
    [nodes]
      uint16_t plane_face_idx
      uint16_t faces_on_plane_count
      uint16_t faces_on_plane_idx_start
      int16_t front_node_idx
      int16_t back_node_idx
    [faces_on_plane]
      uint16_t face_idx (concatenated for all nodes)

All indices are 0-based. -1 (0xFFFF as signed) means no child node.
============================================================================
"""

import sys
import os
import sys
import struct
import subprocess

# ============================================================================
# CONFIGURATION
# ============================================================================
TARGET_SCALE_SIZE = 60.0  # Default scale (can be overridden via command line)


# ============================================================================
# PART 1: OBJ FILTERING AND PARSING
# ============================================================================

def filter_obj_lines(lines):
    """
    Filter OBJ lines keeping only vertices (v) and faces (f)
    Returns (vertex_lines, face_lines)
    """
    vertex_lines = []
    face_lines = []
    for line in lines:
        if line.startswith('v '):
            vertex_lines.append(line)
        elif line.startswith('f '):
            face_lines.append(line)
    return vertex_lines, face_lines


def parse_vertices(vertex_lines):
    """
    Parse vertex lines into list of [x, y, z]
    """
    vertices = []
    for line in vertex_lines:
        parts = line.strip().split()
        x, y, z = map(float, parts[1:4])
        vertices.append([x, y, z])
    return vertices


def parse_faces(face_lines):
    """
    Parse face lines into list of vertex indices (0-based)
    Handles OBJ format: f v1 v2 v3 or f v1/vt1/vn1 v2/vt2/vn2 ...
    """
    faces = []
    for line in face_lines:
        parts = line.strip().split()[1:]
        # OBJ indices start at 1, convert to 0-based
        indices = [int(p.split('/')[0]) - 1 for p in parts]
        faces.append(indices)
    return faces


def read_obj(obj_path):
    """
    Read and parse OBJ file
    Returns (vertices, faces) where:
      - vertices: list of [x, y, z]
      - faces: list of [idx0, idx1, idx2, ...] (0-based indices)
    """
    with open(obj_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    vertex_lines, face_lines = filter_obj_lines(lines)
    vertices = parse_vertices(vertex_lines)
    faces = parse_faces(face_lines)
    
    return vertices, faces


# ============================================================================
# PART 2: CENTERING AND SCALING
# ============================================================================

def center_and_scale(vertices, target_size):
    """
    Center and rescale vertices to fit within target_size
    
    Steps:
    1. Calculate center of mass
    2. Translate all vertices so center is at (0, 0, 0)
    3. Find the largest dimension
    4. Rescale so largest dimension = target_size
    
    Returns transformed vertices
    """
    if not vertices:
        return vertices
    
    n = len(vertices)
    
    # Calculate center of mass
    center = [
        sum(v[0] for v in vertices) / n,
        sum(v[1] for v in vertices) / n,
        sum(v[2] for v in vertices) / n
    ]
    
    # Translate to center
    centered = [
        [v[0] - center[0], v[1] - center[1], v[2] - center[2]]
        for v in vertices
    ]
    
    # Find min/max bounds
    min_vals = [min(v[i] for v in centered) for i in range(3)]
    max_vals = [max(v[i] for v in centered) for i in range(3)]
    
    # Calculate largest dimension
    max_range = max(max_vals[i] - min_vals[i] for i in range(3))
    
    # Calculate scale factor
    scale = target_size / max_range if max_range > 0 else 1.0
    
    # Apply scale
    scaled = [
        [v[0] * scale, v[1] * scale, v[2] * scale]
        for v in centered
    ]
    
    return scaled


def check_fixed32_limits(vertices, limit=32767):
    """
    Verify all coordinates fit within Fixed32 limits [-32768, +32767]
    Returns (is_valid, min_val, max_val)
    """
    if not vertices:
        return True, 0, 0
    
    all_coords = []
    for v in vertices:
        all_coords.extend(v)
    
    min_val = min(all_coords)
    max_val = max(all_coords)
    is_valid = (min_val >= -limit - 1) and (max_val <= limit)
    
    return is_valid, min_val, max_val


# ============================================================================
# PART 3: BSP TREE CONSTRUCTION
# ============================================================================

def face_plane(face, vertices):
    """
    Calculate plane equation (point, normal) for a face
    """
    if len(face) < 3:
        raise ValueError("Face must have at least 3 vertices")
    
    p0 = vertices[face[0]]
    p1 = vertices[face[1]]
    p2 = vertices[face[2]]
    
    # Vectors on the plane (unrolled for speed)
    ux = p1[0] - p0[0]
    uy = p1[1] - p0[1]
    uz = p1[2] - p0[2]
    vx = p2[0] - p0[0]
    vy = p2[1] - p0[1]
    vz = p2[2] - p0[2]
    
    # Normal = cross product (unrolled)
    nx = uy*vz - uz*vy
    ny = uz*vx - ux*vz
    nz = ux*vy - uy*vx
    
    # Normalize
    norm = (nx*nx + ny*ny + nz*nz) ** 0.5
    if norm == 0:
        nx, ny, nz = 0, 0, 1
    else:
        inv_norm = 1.0 / norm
        nx *= inv_norm
        ny *= inv_norm
        nz *= inv_norm
    
    return {'point': p0, 'normal': (nx, ny, nz)}


def classify_face(face, plane, vertices, epsilon=1e-5):
    """
    Classify a face relative to a plane
    Returns: 'front', 'back', 'on', or 'spanning'
    """
    pp = plane['point']
    pn = plane['normal']
    ppx, ppy, ppz = pp[0], pp[1], pp[2]
    pnx, pny, pnz = pn[0], pn[1], pn[2]
    
    front = back = False
    for idx in face:
        p = vertices[idx]
        d = (p[0] - ppx) * pnx + (p[1] - ppy) * pny + (p[2] - ppz) * pnz
        if d > epsilon:
            front = True
        elif d < -epsilon:
            back = True
    
    if front and not back:
        return 'front'
    elif back and not front:
        return 'back'
    elif not front and not back:
        return 'on'
    else:
        return 'spanning'


def build_bsp(faces, vertices, verbose=False):
    """
    Build BSP tree from faces using fully ITERATIVE approach (no recursion)
    Uses explicit stack to avoid Python's recursion limit
    Works with face indices for efficiency
    Returns dict representing BSP node
    """
    if not faces:
        return None
    
    total_faces = len(faces)
    nodes_created = [0]
    
    # Pre-compute all planes indexed by face index
    if verbose:
        print(f"      Pre-computing planes...")
    planes = []
    for face in faces:
        planes.append(face_plane(face, vertices))
    
    # Work with face indices instead of face objects
    face_indices = list(range(total_faces))
    
    # Create root node
    root = {'plane_face': None, 'faces_on_plane': [], 'front': None, 'back': None}
    
    # Stack contains: (parent_node, face_index_list, field_to_set)
    stack = [(None, face_indices, None)]
    
    while stack:
        parent, idx_list, field = stack.pop()
        
        if not idx_list:
            if parent is not None and field is not None:
                parent[field] = None
            continue
        
        # Create node
        node = {'plane_face': None, 'faces_on_plane': [], 'front': None, 'back': None}
        nodes_created[0] += 1
        
        if verbose and nodes_created[0] % 50 == 0:
            print(f"      Nodes: {nodes_created[0]}, stack: {len(stack)}, processing: {len(idx_list)} faces", end='\r')
        
        if parent is None:
            root = node
        else:
            parent[field] = node
        
        # Use first face as splitting plane
        plane_idx = idx_list[0]
        node['plane_face'] = faces[plane_idx]
        
        # Get plane data once
        plane = planes[plane_idx]
        pp = plane['point']
        pn = plane['normal']
        ppx, ppy, ppz = pp[0], pp[1], pp[2]
        pnx, pny, pnz = pn[0], pn[1], pn[2]
        
        idx_on = [plane_idx]  # The plane face itself is always "on"
        idx_front = []
        idx_back = []
        
        epsilon = 1e-5
        
        # Classify remaining faces (skip the plane face!)
        for fi in idx_list[1:]:
            # Inline classification for speed
            front = back = False
            for vidx in faces[fi]:
                p = vertices[vidx]
                d = (p[0] - ppx) * pnx + (p[1] - ppy) * pny + (p[2] - ppz) * pnz
                if d > epsilon:
                    front = True
                    if back:
                        break  # Early exit: spanning
                elif d < -epsilon:
                    back = True
                    if front:
                        break  # Early exit: spanning
            
            if front and back:
                idx_front.append(fi)  # spanning -> front
            elif front:
                idx_front.append(fi)
            elif back:
                idx_back.append(fi)
            else:
                idx_on.append(fi)
        
        node['faces_on_plane'] = [faces[i] for i in idx_on]
        
        # Push children to stack
        if idx_back:
            stack.append((node, idx_back, 'back'))
        else:
            node['back'] = None
            
        if idx_front:
            stack.append((node, idx_front, 'front'))
        else:
            node['front'] = None
    
    if verbose:
        print(f"      Nodes: {nodes_created[0]} total                              ")
    
    return root


# ============================================================================
# PART 4: BSP FLATTENING AND BINARY EXPORT
# ============================================================================

class FlatBSP:
    """
    Flattens recursive BSP tree into arrays for binary export
    """
    def __init__(self):
        self.nodes = []
        self.faces_on_plane = []
        self.faces = []
    
    def flatten(self, root):
        """Iteratively flatten BSP tree (no recursion)"""
        if root is None:
            return -1
        
        # First pass: assign indices to all nodes using BFS
        # Map from id(node) to index
        node_to_idx = {}
        queue = [root]
        idx = 0
        while queue:
            node = queue.pop(0)
            if node is None:
                continue
            if id(node) in node_to_idx:
                continue
            node_to_idx[id(node)] = idx
            self.nodes.append(None)  # Reserve slot
            idx += 1
            if node['front'] is not None:
                queue.append(node['front'])
            if node['back'] is not None:
                queue.append(node['back'])
        
        # Second pass: fill node data using iterative traversal
        stack = [root]
        while stack:
            node = stack.pop()
            if node is None:
                continue
            
            node_idx = node_to_idx[id(node)]
            
            # Skip if already processed
            if self.nodes[node_idx] is not None:
                continue
            
            # Collect faces on this plane
            faces_on = node['faces_on_plane']
            faces_on_start = len(self.faces_on_plane)
            faces_on_count = len(faces_on)
            for f in faces_on:
                self.faces_on_plane.append(f)
            
            # Get children indices
            front_idx = node_to_idx.get(id(node['front']), -1) if node['front'] else -1
            back_idx = node_to_idx.get(id(node['back']), -1) if node['back'] else -1
            
            # Find index of plane face
            plane_face_idx = self.find_face_idx(node['plane_face'])
            
            # Store node data
            self.nodes[node_idx] = {
                'plane_face_idx': plane_face_idx,
                'faces_on_plane_count': faces_on_count,
                'faces_on_plane_idx_start': faces_on_start,
                'front_node_idx': front_idx,
                'back_node_idx': back_idx
            }
            
            # Push children
            if node['back'] is not None:
                stack.append(node['back'])
            if node['front'] is not None:
                stack.append(node['front'])
        
        return 0  # Root index
    
    def find_face_idx(self, face):
        """Find index of face in faces list"""
        for i, f in enumerate(self.faces):
            if f == face:
                return i
        return 0xFFFF
    
    def run(self, bsp_tree, faces):
        """Execute flattening"""
        self.faces = faces
        self.flatten(bsp_tree)


def write_bsp_binary(path, vertices, faces, flat_bsp):
    """
    Write BSP data to binary file for Apple IIGS
    """
    with open(path, 'wb') as f:
        # Header: vertex_count, face_count, node_count
        f.write(struct.pack('<HHH', len(vertices), len(faces), len(flat_bsp.nodes)))
        
        # Vertices: float32 x, y, z
        for v in vertices:
            f.write(struct.pack('<fff', v[0], v[1], v[2]))
        
        # Faces: uint8 vertex_count, then uint16 indices
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
        
        # Faces on plane (indices)
        for face in flat_bsp.faces_on_plane:
            idx = flat_bsp.find_face_idx(face)
            f.write(struct.pack('<H', idx))


# ============================================================================
# MAIN PIPELINE
# ============================================================================

def convert_obj_to_bsp(input_obj, output_bin, target_size=None, verbose=True):
    """
    Complete OBJ to BSP conversion pipeline
    
    Parameters:
        input_obj: Path to input OBJ file
        output_bin: Path to output binary BSP file
        target_size: Scale target (default: TARGET_SCALE_SIZE)
        verbose: Print progress messages
    
    Returns:
        True if successful, False otherwise
    """
    if target_size is None:
        target_size = TARGET_SCALE_SIZE
    
    # Step 1: Read and parse OBJ
    if verbose:
        print(f"\n[1/5] Reading OBJ: {input_obj}")
    
    try:
        vertices, faces = read_obj(input_obj)
    except Exception as e:
        print(f"Error reading OBJ: {e}")
        return False
    
    if verbose:
        print(f"      Vertices: {len(vertices)}")
        print(f"      Faces: {len(faces)}")
    
    # Step 2: Center and scale
    if verbose:
        print(f"\n[2/5] Centering and scaling (target: {target_size})")
    
    vertices = center_and_scale(vertices, target_size)
    
    is_valid, min_val, max_val = check_fixed32_limits(vertices)
    if verbose:
        print(f"      Range: [{min_val:.2f}, {max_val:.2f}]")
        if is_valid:
            print(f"      ✓ Within Fixed32 limits")
        else:
            print(f"      ✗ WARNING: Exceeds Fixed32 limits!")
    
    # Step 3: Build BSP tree
    if verbose:
        print(f"\n[3/5] Building BSP tree...")
    
    bsp_tree = build_bsp(faces, vertices, verbose=verbose)
    if verbose:
        print()  # New line after progress
    
    # Step 4: Flatten BSP tree
    if verbose:
        print(f"\n[4/5] Flattening BSP tree...")
    
    flat_bsp = FlatBSP()
    flat_bsp.run(bsp_tree, faces)
    
    if verbose:
        print(f"      Nodes: {len(flat_bsp.nodes)}")
        print(f"      Faces on plane entries: {len(flat_bsp.faces_on_plane)}")
    
    # Step 5: Write binary file
    if verbose:
        print(f"\n[5/5] Writing binary BSP: {output_bin}")
    
    try:
        write_bsp_binary(output_bin, vertices, faces, flat_bsp)
    except Exception as e:
        print(f"Error writing binary: {e}")
        return False
    
    # Calculate file size
    file_size = os.path.getsize(output_bin)
    if verbose:
        print(f"      File size: {file_size} bytes")
        print(f"\n✓ Conversion complete!")
    
    return True


# ============================================================================
# CADIUS DEPLOYMENT
# ============================================================================

# Configuration for Cadius deployment
APPLE_DISK_PATH = "F:\\Bruno\\Dev\\AppleWin\\GS\\activeGS\\Live.Install.po"
PRODOS_DIR = "/LIVE.INSTALL/"


def check_cadius():
    """Check if Cadius is available in PATH"""
    try:
        result = subprocess.run("where Cadius.exe", shell=True, 
                              capture_output=True, text=True, check=True)
        return True
    except subprocess.CalledProcessError:
        return False


def is_valid_prodos_name(name):
    """
    Check if a filename is valid for ProDOS
    Rules:
    - Max 15 characters
    - Only letters, numbers, and periods
    - Must start with a letter
    - No underscores or special characters
    """
    if not name or len(name) > 15:
        return False
    if not name[0].isalpha():
        return False
    for c in name:
        if not (c.isalnum() or c == '.'):
            return False
    return True


def deploy_to_disk(file_path, verbose=True):
    """
    Deploy a file to Apple disk image using Cadius
    
    Parameters:
        file_path: Path to the file to deploy
        verbose: Print progress messages
    
    Returns:
        True if successful, False otherwise
    """
    if not os.path.exists(file_path):
        print(f"Error: File not found: {file_path}")
        return False
    
    if not os.path.exists(APPLE_DISK_PATH):
        print(f"Error: Apple disk image not found: {APPLE_DISK_PATH}")
        return False
    
    if not check_cadius():
        print("Error: Cadius.exe not found in PATH!")
        return False
    
    filename = os.path.basename(file_path)
    
    # Validate ProDOS filename
    if not is_valid_prodos_name(filename):
        print(f"Error: Invalid ProDOS filename: {filename}")
        print("  - Max 15 characters")
        print("  - Only letters, numbers, and periods")
        print("  - Must start with a letter")
        print("  - No underscores or special characters")
        return False
    
    if verbose:
        print(f"\n[6/6] Deploying to Apple disk image...")
        print(f"      Disk: {APPLE_DISK_PATH}")
        print(f"      Directory: {PRODOS_DIR}")
        print(f"      File: {filename}")
    
    # Delete existing file (ignore errors)
    if verbose:
        print(f"      Removing existing file...")
    result = subprocess.run(
        f'Cadius.exe DELETEFILE "{APPLE_DISK_PATH}" {PRODOS_DIR}{filename}',
        shell=True, capture_output=True, text=True
    )
    
    # Add new file
    if verbose:
        print(f"      Adding file...")
    result = subprocess.run(
        f'Cadius.exe ADDFILE "{APPLE_DISK_PATH}" {PRODOS_DIR} "{file_path}"',
        shell=True, capture_output=True, text=True
    )
    
    output = result.stdout + result.stderr
    if "error" in output.lower() and "file not found" not in output.lower():
        print(f"Error: Cadius failed!")
        print(output)
        return False
    
    if verbose:
        print(f"      ✓ Deployed successfully!")
    
    return True


def main():
    """Main entry point"""
    if len(sys.argv) < 3:
        print(__doc__)
        print("Error: Missing arguments")
        print("\nUsage: python obj_to_bsp.py <input.obj> <output.bin> [target_size] [-d]")
        sys.exit(1)
    
    # Parse arguments
    args = sys.argv[1:]
    deploy = False
    
    # Check for -d or --deploy flag
    if '-d' in args:
        deploy = True
        args.remove('-d')
    if '--deploy' in args:
        deploy = True
        args.remove('--deploy')
    
    if len(args) < 2:
        print("Error: Missing input or output file")
        sys.exit(1)
    
    input_obj = args[0]
    output_bin = args[1]
    
    # Optional target size
    target_size = TARGET_SCALE_SIZE
    if len(args) > 2:
        try:
            target_size = float(args[2])
        except ValueError:
            print(f"Invalid target_size: {args[2]}")
            sys.exit(1)
    
    # Check input file exists
    if not os.path.exists(input_obj):
        print(f"Error: Input file not found: {input_obj}")
        sys.exit(1)
    
    # Run conversion
    success = convert_obj_to_bsp(input_obj, output_bin, target_size, verbose=True)
    
    if not success:
        sys.exit(1)
    
    # Deploy to disk if requested
    if deploy:
        if not deploy_to_disk(output_bin, verbose=True):
            sys.exit(1)
    
    sys.exit(0)


if __name__ == '__main__':
    main()
