import json, math, os, numpy as np

src = "/mnt/data/2026-rebuilt-andymark.json"
with open(src,"r") as f:
    data=json.load(f)

field_len=float(data["field"]["length"])
field_wid=float(data["field"]["width"])

tags=data["tags"]

out_dir="/mnt/data/frc2026_ros2_map"
os.makedirs(out_dir, exist_ok=True)

# 1) Blank occupancy grid map (PGM + YAML) for nav2 map_server
resolution=0.05  # m/pixel (5 cm)
width_px=int(math.ceil(field_len/resolution))
height_px=int(math.ceil(field_wid/resolution))

# occupancy: 0=free (white 254), 100=occupied (black 0), -1=unknown (205)
# We'll make everything inside field free.
img = np.full((height_px, width_px), 254, dtype=np.uint8)

pgm_path=os.path.join(out_dir,"frc2026_field_blank.pgm")
with open(pgm_path,"wb") as f:
    header=f"P5\n# FRC 2026 field blank map\n{width_px} {height_px}\n255\n"
    f.write(header.encode("ascii"))
    f.write(img.tobytes())

yaml_path=os.path.join(out_dir,"frc2026_field_blank.yaml")
# origin: bottom-left corner at (0,0), and we set y-axis up in image; map_server assumes origin is lower-left of image.
# With PGM, row 0 is top; map_server expects image origin at lower-left but handles via origin + neg Y? Actually it uses standard ROS map: pixel (0,0) at lower-left.
# To avoid confusion, set origin to (0,0) and accept that visualization is consistent for coordinates, although image vertical axis may appear flipped depending on viewer.
map_yaml={
    "image": os.path.basename(pgm_path),
    "resolution": resolution,
    "origin": [0.0, 0.0, 0.0],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.196
}
with open(yaml_path,"w") as f:
    for k,v in map_yaml.items():
        f.write(f"{k}: {json.dumps(v) if isinstance(v,(list,dict,str)) else v}\n")

# 2) AprilTag landmarks YAML (generic) + apriltag_ros style YAML
# Default tag size: 6.5 inches = 0.1651 m (user can edit)
tag_size_m=0.1651

apriltag_ros_yaml_path=os.path.join(out_dir,"frc2026_apriltag_ros.yaml")
apriltag_ros_cfg={
    "tag_family": "tag36h11",
    "tag_size": tag_size_m,
    "standalone_tags": [],
}
for t in tags:
    tid=int(t["ID"])
    tr=t["pose"]["translation"]
    q=t["pose"]["rotation"]["quaternion"]
    apriltag_ros_cfg["standalone_tags"].append({
        "id": tid,
        "size": tag_size_m,
        "pose": {
            "position": {"x": float(tr["x"]), "y": float(tr["y"]), "z": float(tr["z"])},
            "orientation": {"x": float(q["X"]), "y": float(q["Y"]), "z": float(q["Z"]), "w": float(q["W"])},
        }
    })
with open(apriltag_ros_yaml_path,"w") as f:
    f.write("# FRC 2026 AprilTags converted from WPILib layout JSON\n")
    f.write("# Note: edit tag_family and tag_size/size if your pipeline differs.\n")
    f.write("apriltag_ros:\n")
    f.write("  ros__parameters:\n")
    f.write(f"    tag_family: {apriltag_ros_cfg['tag_family']}\n")
    f.write(f"    tag_size: {apriltag_ros_cfg['tag_size']}\n")
    f.write("    standalone_tags:\n")
    for st in apriltag_ros_cfg["standalone_tags"]:
        f.write(f"      - id: {st['id']}\n")
        f.write(f"        size: {st['size']}\n")
        p=st["pose"]["position"]; o=st["pose"]["orientation"]
        f.write("        pose:\n")
        f.write(f"          position: {{x: {p['x']}, y: {p['y']}, z: {p['z']}}}\n")
        f.write(f"          orientation: {{x: {o['x']}, y: {o['y']}, z: {o['z']}, w: {o['w']}}}\n")

# 3) Static TF publishers list (CSV-ish YAML) for scripting
tf_list_path=os.path.join(out_dir,"frc2026_tag_static_tfs.yaml")
with open(tf_list_path,"w") as f:
    f.write("# Each entry is a static transform from 'map' -> 'tag_<ID>'\n")
    f.write("static_transforms:\n")
    for t in tags:
        tid=int(t["ID"])
        tr=t["pose"]["translation"]
        q=t["pose"]["rotation"]["quaternion"]
        f.write(f"  - parent: map\n")
        f.write(f"    child: tag_{tid}\n")
        f.write(f"    translation: [{float(tr['x'])}, {float(tr['y'])}, {float(tr['z'])}]\n")
        f.write(f"    rotation_xyzw: [{float(q['X'])}, {float(q['Y'])}, {float(q['Z'])}, {float(q['W'])}]\n")

# Zip outputs for easy download
import zipfile, pathlib
zip_path="/mnt/data/frc2026_ros2_map_bundle.zip"
with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as z:
    for p in pathlib.Path(out_dir).rglob("*"):
        z.write(p, arcname=str(p.relative_to(out_dir)))

(zip_path, out_dir, (field_len, field_wid, width_px, height_px, resolution), len(tags))
