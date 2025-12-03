import sys
import math
import numpy as np

def fixed_to_float(val):
    return val / 65536.0

def float_to_fixed(val):
    return int(round(val * 65536.0))

def deg_to_rad(deg):
    return deg * math.pi / 180.0

def transform_and_project(vertices, params):
    angle_h = deg_to_rad(params['angle_h'])
    angle_v = deg_to_rad(params['angle_v'])
    angle_w = deg_to_rad(params['angle_w'])
    distance = params['distance']
    centre_x = params.get('centre_x', 160)
    centre_y = params.get('centre_y', 100)
    scale = 100.0
    cos_h = math.cos(angle_h)
    sin_h = math.sin(angle_h)
    cos_v = math.cos(angle_v)
    sin_v = math.sin(angle_v)
    cos_w = math.cos(angle_w)
    sin_w = math.sin(angle_w)
    cos_h_cos_v = cos_h * cos_v
    sin_h_cos_v = sin_h * cos_v
    cos_h_sin_v = cos_h * sin_v
    sin_h_sin_v = sin_h * sin_v
    results = []
    for i, (x, y, z) in enumerate(vertices):
        term1 = x * cos_h_cos_v
        term2 = y * sin_h_cos_v
        term3 = z * sin_v
        zo = -term1 - term2 - term3 + distance
        if zo > 0:
            xo = -x * sin_h + y * cos_h
            yo = -x * cos_h_sin_v - y * sin_h_sin_v + z * cos_v
            inv_zo = scale / zo
            x2d_temp = xo * inv_zo + centre_x
            y2d_temp = centre_y - yo * inv_zo
            x2d = int(round(cos_w * (x2d_temp - centre_x) - sin_w * (centre_y - y2d_temp) + centre_x))
            y2d = int(round(centre_y - (sin_w * (x2d_temp - centre_x) + cos_w * (centre_y - y2d_temp))))
        else:
            xo = yo = 0.0
            x2d = y2d = -1
        results.append((x2d, y2d, xo, yo, zo))
    return results

def parse_obj(filename):
    vertices = []
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.strip().split()
                x, y, z = map(float, parts[1:4])
                vertices.append((x, y, z))
    return vertices

def main():
    if len(sys.argv) < 2:
        print("Usage: python compare_projection.py objfile [angle_h] [angle_v] [angle_w] [distance]")
        sys.exit(1)
    objfile = sys.argv[1]
    vertices = parse_obj(objfile)
    # Default params (same as GS3Df)
    params = {
        'angle_h': float(sys.argv[2]) if len(sys.argv) > 2 else 30.0,
        'angle_v': float(sys.argv[3]) if len(sys.argv) > 3 else 30.0,
        'angle_w': float(sys.argv[4]) if len(sys.argv) > 4 else 0.0,
        'distance': float(sys.argv[5]) if len(sys.argv) > 5 else 30.0,
        'centre_x': 160,
        'centre_y': 100
    }
    results = transform_and_project(vertices, params)
    with open('projection_pc.log', 'w') as f:
        f.write(f"# Model file: {objfile}\n")
        f.write(f"# Observer params: angle_h={params['angle_h']}, angle_v={params['angle_v']}, angle_w={params['angle_w']}, distance={params['distance']}\n")
        f.write("Vertex\tx2d\ty2d\txo\tyo\tzo\n")
        for i, (x2d, y2d, xo, yo, zo) in enumerate(results):
            f.write(f"{i}\t{x2d}\t{y2d}\t{xo:.6f}\t{yo:.6f}\t{zo:.6f}\n")
    print("Projection log written to projection_pc.log")

if __name__ == '__main__':
    main()
