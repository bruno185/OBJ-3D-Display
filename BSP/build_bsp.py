import sys
import os
import json

# ============================================================================
# BSP Tree Construction for OBJ Files
# ============================================================================
# This script reads an OBJ file (vertices + faces), builds a BSP tree, and
# exports the tree as a JSON file (bsp.json) for later use on Apple IIGS or elsewhere.
#
# Usage:
#   python build_bsp.py <input.obj> [output.json]
#
# The output JSON contains:
#   - vertices: list of [x, y, z]
#   - faces: list of [i1, i2, i3, ...]
#   - bsp_tree: recursive BSP structure


def parse_obj(obj_path):
    """
    Parse OBJ file, return (vertices, faces)
    Vertices: list of [x, y, z]
    Faces: list of lists of vertex indices (0-based)
    """
    vertices = []
    faces = []
    with open(obj_path, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.strip().split()
                x, y, z = map(float, parts[1:4])
                vertices.append([x, y, z])
            elif line.startswith('f '):
                parts = line.strip().split()[1:]
                # OBJ indices start at 1
                indices = [int(p.split('/')[0]) - 1 for p in parts]
                faces.append(indices)
    return vertices, faces


def face_plane(face, vertices):
    """
    Return a plane equation (point, normal) for a face (assume triangle or quad)
    """
    if len(face) < 3:
        raise ValueError("Face must have at least 3 vertices")
    p0 = vertices[face[0]]
    p1 = vertices[face[1]]
    p2 = vertices[face[2]]
    # Compute normal (cross product)
    u = [p1[i] - p0[i] for i in range(3)]
    v = [p2[i] - p0[i] for i in range(3)]
    n = [
        u[1]*v[2] - u[2]*v[1],
        u[2]*v[0] - u[0]*v[2],
        u[0]*v[1] - u[1]*v[0]
    ]
    # Normalize
    norm = sum(x*x for x in n) ** 0.5
    if norm == 0:
        n = [0, 0, 1]
    else:
        n = [x / norm for x in n]
    return {'point': p0, 'normal': n}


def classify_face(face, plane, vertices, epsilon=1e-5):
    """
    Classify a face as 'front', 'back', or 'on' relative to a plane
    """
    front = back = False
    for idx in face:
        p = vertices[idx]
        d = sum((p[i] - plane['point'][i]) * plane['normal'][i] for i in range(3))
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
        return 'spanning'  # Not handled in this simple version


def build_bsp(faces, vertices):
    """
    Recursively build a BSP tree from faces
    Returns a dict representing the BSP node
    """
    if not faces:
        return None
    # Pick the first face as the splitting plane
    plane_face = faces[0]
    plane = face_plane(plane_face, vertices)
    faces_on = []
    faces_front = []
    faces_back = []
    for face in faces:
        cls = classify_face(face, plane, vertices)
        if cls == 'on':
            faces_on.append(face)
        elif cls == 'front':
            faces_front.append(face)
        elif cls == 'back':
            faces_back.append(face)
        else:
            # For simplicity, assign spanning faces to front (real BSP would split)
            faces_front.append(face)
    return {
        'plane_face': plane_face,
        'faces_on_plane': faces_on,
        'front': build_bsp(faces_front, vertices),
        'back': build_bsp(faces_back, vertices)
    }


def export_bsp_json(vertices, faces, bsp_tree, out_path):
    """
    Export the BSP tree and geometry as JSON
    """
    data = {
        'vertices': vertices,
        'faces': faces,
        'bsp_tree': bsp_tree
    }
    with open(out_path, 'w') as f:
        json.dump(data, f, indent=2)


def main():
    if len(sys.argv) < 2:
        print("Usage: python build_bsp.py <input.obj> [output.json]")
        sys.exit(1)
    obj_path = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else 'bsp.json'
    print(f"Reading OBJ: {obj_path}")
    vertices, faces = parse_obj(obj_path)
    print(f"  {len(vertices)} vertices, {len(faces)} faces")
    print("Building BSP tree...")
    bsp_tree = build_bsp(faces, vertices)
    print("Exporting BSP tree to:", out_path)
    export_bsp_json(vertices, faces, bsp_tree, out_path)
    print("Done.")

if __name__ == '__main__':
    main()
