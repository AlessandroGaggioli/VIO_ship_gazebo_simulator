#!/usr/bin/env python3

"""
this script computes the dimensions of an STL file (min, max, size, center) for each axis.
It can handle both ASCII and binary STL files.
Usage: python STL_dimension.py file.stl
"""
import struct
import sys
import os

def is_binary_stl(filename):
    with open(filename, 'rb') as f:
        start = f.read(80)
    return not start.lower().startswith(b"solid")


def stl_bounds_binary(filename):
    min_x = min_y = min_z = float('inf')
    max_x = max_y = max_z = float('-inf')

    with open(filename, 'rb') as f:
        f.read(80)
        tri_count_bytes = f.read(4)

        if len(tri_count_bytes) < 4:
            raise ValueError("File too small to be a valid binary STL")

        n_triangles = struct.unpack('<I', tri_count_bytes)[0]

        for _ in range(n_triangles):
            chunk = f.read(50)
            if len(chunk) < 50:
                break  # fallback if file is truncated

            for i in range(3):
                x, y, z = struct.unpack('<3f', chunk[12 + i*12 : 24 + i*12])

                min_x = min(min_x, x)
                min_y = min(min_y, y)
                min_z = min(min_z, z)

                max_x = max(max_x, x)
                max_y = max(max_y, y)
                max_z = max(max_z, z)

    return (min_x, max_x, min_y, max_y, min_z, max_z)


def stl_bounds_ascii(filename):
    min_x = min_y = min_z = float('inf')
    max_x = max_y = max_z = float('-inf')

    with open(filename, 'r', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if line.startswith("vertex"):
                try:
                    _, x, y, z = line.split()
                    x, y, z = float(x), float(y), float(z)
                except:
                    continue

                min_x = min(min_x, x)
                min_y = min(min_y, y)
                min_z = min(min_z, z)

                max_x = max(max_x, x)
                max_y = max(max_y, y)
                max_z = max(max_z, z)

    return (min_x, max_x, min_y, max_y, min_z, max_z)


def compute_bounds(filename):
    if is_binary_stl(filename):
        print("Detected: BINARY STL")
        return stl_bounds_binary(filename)
    else:
        print("Detected: ASCII STL")
        return stl_bounds_ascii(filename)


# ---- main ----

if len(sys.argv) < 2:
    print("Uso: python script.py file.stl")
    sys.exit(1)

file = sys.argv[1]

min_x, max_x, min_y, max_y, min_z, max_z = compute_bounds(file)

size_x = max_x - min_x
size_y = max_y - min_y
size_z = max_z - min_z

print(f'X: min={min_x:.3f} max={max_x:.3f} size={size_x:.3f} center={(min_x+max_x)/2:.3f}')
print(f'Y: min={min_y:.3f} max={max_y:.3f} size={size_y:.3f} center={(min_y+max_y)/2:.3f}')
print(f'Z: min={min_z:.3f} max={max_z:.3f} size={size_z:.3f} center={(min_z+max_z)/2:.3f}')


