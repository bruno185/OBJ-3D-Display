import sys
import os

# ============================================================================
# CONFIGURATION
# ============================================================================
# Adjust TARGET_SCALE_SIZE value to control the scaling of the 3D model
# Larger values = bigger model on screen
# Typical range: 20-50 for Apple IIGS display
# Make sure the model fits within Fixed32 limits [-32768, +32767]
TARGET_SCALE_SIZE = 60.0
# This section filters OBJ files to keep only essential elements:
# vertices (v), faces (f), and comments (#)
# Other lines (textures, normals, etc.) are removed

def filter_obj_file(input_path, output_path):
    """
    Filter an OBJ file keeping only:
    - 'v ' lines (vertices)
    - 'f ' lines (faces/triangles)
    - '#' comments
    
    Parameters:
        input_path: path to OBJ file to read
        output_path: path to filtered OBJ file to write
    
    Returns:
        number of lines removed
    """
    total = 0
    kept = 0
    with open(input_path, "r", encoding="utf-8") as infile, open(output_path, "w", encoding="utf-8") as outfile:
        for line in infile:
            total += 1
            # Keep only v (vertices), f (faces), # (comments)
            if line.startswith(("v ", "f ", "#")):
                outfile.write(line)
                kept += 1
    removed = total - kept
    return removed

def count_lines_v_f(file_path):
    """
    Count vertex (v) and face (f) lines in an OBJ file
    
    Parameters:
        file_path: path to OBJ file
    
    Returns:
        (vertex_count, face_count)
    """
    v_count = 0  # Counter for vertices (points)
    f_count = 0  # Counter for faces (triangles)
    with open(file_path, "r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("v "):
                v_count += 1
            elif line.startswith("f "):
                f_count += 1
    return v_count, f_count

# ============================================================================
# CENTERING AND SCALING FUNCTIONS
# ============================================================================
# These functions transform 3D model coordinates so that it:
# 1. Is centered at the origin (0, 0, 0)
# 2. Fits within a target size (e.g., 30 units)
# This ensures the model fits within Fixed32 limits on the Apple IIGS

def read_vertices(obj_path):
    """
    Read all vertices from an OBJ file
    
    Parameters:
        obj_path: path to OBJ file
    
    Returns:
        list of vertices [[x1, y1, z1], [x2, y2, z2], ...]
    """
    vertices = []
    with open(obj_path, 'r') as f:
        for line in f:
            if line.startswith('v '):
                # Extract the three coordinates x, y, z from the line
                parts = line.strip().split()
                x, y, z = map(float, parts[1:4])
                vertices.append([x, y, z])
    return vertices

def write_obj(obj_path, vertices, other_lines):
    """
    Write vertices and other lines to an OBJ file
    
    Parameters:
        obj_path: path to OBJ file to write
        vertices: list of vertices to write
        other_lines: other lines (faces, comments) to preserve
    """
    with open(obj_path, 'w') as f:
        # Write all vertices
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        # Write faces and comments
        for line in other_lines:
            f.write(line)

def center_and_scale(vertices, target_size=None):
    """
    Center and rescale vertices (pure Python, no numpy)
    
    Steps:
    1. Calculate the center of mass of the model
    2. Translate all vertices so the center is at (0, 0, 0)
    3. Find the largest dimension
    4. Rescale so the largest dimension = target_size
    
    Parameters:
        vertices: list of vertices [[x, y, z], ...]
        target_size: target size after rescaling (default: uses TARGET_SCALE_SIZE)
    
    Returns:
        transformed vertices
    """
    if target_size is None:
        target_size = TARGET_SCALE_SIZE
    
    if not vertices:
        return vertices
    
    # STEP 1: Calculate center of mass
    # Center = average of all vertices
    n = len(vertices)
    center = [
        sum(v[0] for v in vertices) / n,  # Average of X
        sum(v[1] for v in vertices) / n,  # Average of Y
        sum(v[2] for v in vertices) / n   # Average of Z
    ]
    
    # STEP 2: Translation to center
    # Subtract the center from each vertex
    centered = [
        [v[0] - center[0], v[1] - center[1], v[2] - center[2]]
        for v in vertices
    ]
    
    # STEP 3: Find min/max bounds after centering
    # For each axis (X, Y, Z), find minimum and maximum values
    min_vals = [min(v[i] for v in centered) for i in range(3)]
    max_vals = [max(v[i] for v in centered) for i in range(3)]
    
    # Calculate the largest dimension (width, height, or depth)
    # max_range = max([width_X, height_Y, depth_Z])
    max_range = max(max_vals[i] - min_vals[i] for i in range(3))
    
    # STEP 4: Calculate scale factor
    # scale = target_size / max_dimension
    # Allows rescaling the model to the target size
    scale = target_size / max_range if max_range > 0 else 1.0
    
    # STEP 5: Apply scale to all vertices
    scaled = [
        [v[0] * scale, v[1] * scale, v[2] * scale]
        for v in centered
    ]
    
    return scaled


# ============================================================================
# FIXED32 LIMIT VALIDATION
# ============================================================================
# Fixed32 = 16.16 fixed-point format (16-bit integer + 16-bit decimal)
# Valid range: [-32768, +32767]
# This section verifies that ALL coordinates fit within these limits
# Critical for Apple IIGS which uses Fixed32

def check_fixed32_limits(vertices, limit=32767):
    """
    Verify that all coordinates fit within Fixed32 limits
    Range: [-32768, +32767]
    
    Parameters:
        vertices: list of vertices [[x, y, z], ...]
        limit: positive limit (default: 32767)
    
    Returns:
        tuple: (is_valid, min_val, max_val, out_of_range_count)
        - is_valid: True if all coordinates are within limits
        - min_val: minimum value found
        - max_val: maximum value found
        - out_of_range_count: number of invalid coordinates
    """
    # Collect all coordinates into a single list
    all_coords = []
    for v in vertices:
        all_coords.extend(v)  # Add x, y, z
    
    # Special case: no coordinates
    if not all_coords:
        return True, 0, 0, 0
    
    # Find min and max
    min_val = min(all_coords)
    max_val = max(all_coords)
    
    # Count how many coordinates exceed the limits
    out_of_range = sum(1 for c in all_coords if c < -limit-1 or c > limit)
    
    # Verify all coordinates are within [-32768, +32767]
    is_valid = (min_val >= -limit-1) and (max_val <= limit)
    
    return is_valid, min_val, max_val, out_of_range

def process_obj(input_path, output_path, target_size=None, verbose=True):
    """
    Complete OBJ processing pipeline:
    1. Filter OBJ file
    2. Center and scale vertices
    3. Verify Fixed32 limits
    4. Write output
    
    Parameters:
        input_path: path to input OBJ file
        output_path: path to output OBJ file
        target_size: scaling target size (default: uses TARGET_SCALE_SIZE)
        verbose: print detailed progress messages (default: True)
    
    Returns:
        True if all coordinates are within Fixed32 limits, False otherwise
    """
    if target_size is None:
        target_size = TARGET_SCALE_SIZE
    
    # STEP 1: Filter the OBJ file to remove unwanted elements
    # Creates a temporary file with only v, f, and # lines
    temp_filtered = input_path + ".temp_filtered"
    removed = filter_obj_file(input_path, temp_filtered)
    
    if verbose:
        v_count, f_count = count_lines_v_f(temp_filtered)
        print(f"\n[FILTERING]")
        print(f"  Lines removed: {removed}")
        print(f"  Vertices: {v_count}")
        print(f"  Faces: {f_count}")
    
    # STEP 2: Read vertices and other lines from filtered file
    vertices = []
    other_lines = []
    with open(temp_filtered, 'r') as f:
        for line in f:
            if line.startswith('v '):
                # Parse vertex coordinates
                parts = line.strip().split()
                x, y, z = map(float, parts[1:4])
                vertices.append([x, y, z])
            else:
                # Keep faces and comments
                other_lines.append(line)
    
    # STEP 3: Center and rescale all vertices
    # Find original range before transformation
    original_min = min([min(v) for v in vertices]) if vertices else 0
    original_max = max([max(v) for v in vertices]) if vertices else 0
    
    # Apply center and scale transformation
    new_vertices = center_and_scale(vertices, target_size)
    
    if verbose:
        print(f"\n[CENTERING & SCALING]")
        print(f"  Original range: [{original_min:.6f}, {original_max:.6f}]")
        new_min = min([min(v) for v in new_vertices]) if new_vertices else 0
        new_max = max([max(v) for v in new_vertices]) if new_vertices else 0
        print(f"  After scaling:  [{new_min:.6f}, {new_max:.6f}]")
        print(f"  Target size: {target_size}")
    
    # STEP 4: Validate that all coordinates fit within Fixed32 range
    # This is critical for Apple IIGS compatibility
    is_valid, min_val, max_val, out_of_range = check_fixed32_limits(new_vertices)
    
    if verbose:
        print(f"\n[FIXED32 VALIDATION]")
        print(f"  Range: [{min_val:.6f}, {max_val:.6f}]")
        print(f"  Fixed32 limits: [-32768, +32767]")
        if is_valid:
            print(f"  ✅ ALL COORDINATES WITHIN LIMITS")
        else:
            print(f"  ❌ {out_of_range} COORDINATES OUT OF RANGE!")
    
    # STEP 5: Write the transformed model to output file
    write_obj(output_path, new_vertices, other_lines)
    
    if verbose:
        print(f"\n[OUTPUT]")
        print(f"  Saved to: {output_path}")
    
    # Clean up temporary filtered file
    if os.path.exists(temp_filtered):
        os.remove(temp_filtered)
    
    return is_valid

if __name__ == '__main__':
    """
    Main entry point for the OBJ processing script
    
    Usage:
        python center_scale_obj.py <input.obj> [output.obj] [target_size]
    
    Arguments:
        input.obj:   Path to input OBJ file (required)
        output.obj:  Path to output OBJ file (optional, default: input_scaled.obj)
        target_size: Scaling target size (optional, default: 30.0)
    
    Example:
        python center_scale_obj.py car2.obj car2scaled.obj 30
    
    Returns:
        0 if all coordinates are within Fixed32 limits
        1 if any coordinate exceeds the limits
    """
    if len(sys.argv) < 2:
        print("Usage: python center_scale_obj.py <input.obj> [output.obj] [target_size]")
        print("  input.obj:   Path to input OBJ file")
        print("  output.obj:  Path to output OBJ file (default: input_scaled.obj)")
        print("  target_size: Scaling target (default: 30.0)")
        sys.exit(1)
    
    # Get input file path (required)
    input_path = sys.argv[1]
    
    # Get output file path or generate default
    if len(sys.argv) > 2:
        output_path = sys.argv[2]
    else:
        # Default: append "scaled" before the extension
        # Example: car2.obj -> car2scaled.obj
        base, ext = os.path.splitext(input_path)
        output_path = f"{base}scaled{ext}"
    
    # Get target size or use default
    target_size = TARGET_SCALE_SIZE  # Use the global configuration value
    if len(sys.argv) > 3:
        try:
            target_size = float(sys.argv[3])
        except ValueError:
            print(f"Invalid target_size: {sys.argv[3]}")
            sys.exit(1)
    
    # Run the full OBJ processing pipeline
    is_valid = process_obj(input_path, output_path, target_size, verbose=True)
    
    # Exit with status code: 0 if successful, 1 if validation failed
    sys.exit(0 if is_valid else 1)
