import trimesh
import os
import glob

# Grab every single .stl file in the folder
for filename in glob.glob('*.stl'):
    # Skip it if it's already a collision file we generated
    if '_collision' in filename:
        continue
        
    print(f"Processing {filename}...")
    try:
        # Load the mesh, shrink-wrap it, and save it
        mesh = trimesh.load(filename)
        collision_hull = mesh.convex_hull
        
        name, ext = os.path.splitext(filename)
        collision_hull.export(f"{name}_collision{ext}")
        
    except Exception as e:
        print(f"  -> Failed: {e}")

print("Done generating all collision hulls!")