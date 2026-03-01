# arena_tag_map

## Description

Package to house the arena definition (geometry + coordinate convention + AprilTag ground-truth poses) 

This package is provides

    - `arena_tag_map/` → source of truth for the arena coordinate frame + tag layout

Given that we have “dimensions known” + “tag position/type known”, the minimum viable “arena map” for tag localization is a tag layout file in arena_tag_map/config/ is the best path (versus SLAM type localization).

## Arena map

Minimum for AprilTag-based localization

✅ arena_tag_map/config/arena_tags.yaml

This defines:
    - map frame convention (implicitly by how you place poses)
    - tag IDs/family/size
    - each tag’s pose in map

## Template: 
- `arena_tag_map/config/arena.yaml` (recommended “arena + tags” schema)
    - template makes the coordinate system explicit and keeps everything in one place.

- `tag_pose_observer/observer_node.py` uses `load_tag_map()` which expects `tags:` at the top level. If you adopt the more explicit schema above (apriltags: tags:), update the loader like this:
    - read data["apriltags"]["tags"] instead of data["tags"]
    - read data["apriltags"]["default_size_m"] if you later want to use size in gating

## Author Tag Positions

### How to author tag positions with center origin

- Convert from “distance from a corner” to center-origin by subtracting half extents:
    - x_center = x_from_west - x_half
    - y_center = y_from_south - y_half

- Wall planes:
    - West wall: x = -x_half
    - East wall: x = +x_half
    - South wall: y = -y_half
    - North wall: y = +y_half


CMakeLists.txt

package.xml
- install config/ and scripts/ so get_package_share_directory("arena_tag_map") works in launch files. 


[TODO] Take FRC files and turn them into a single file like `757-lab_arena_tags.yaml`

## Generate World And Tag Maps

Use the generator script to build:
- a Nav2 map image + yaml (`.pgm` + `.yaml`)
- an AprilTag layout yaml (`*_arena_tags.yaml`)

Input spec template:
- `arena_tag_map/config/generate_map_request_example.yaml`

Run from the workspace root:

```bash
python3 src/arena_tag_map/scripts/generate_world_and_tag_maps.py \
  --input src/arena_tag_map/config/generate_map_request_example.yaml \
  --output-dir src/arena_tag_map/config \
  --basename practice_field
```

Or after `colcon build`:

```bash
ros2 run arena_tag_map generate_world_and_tag_maps.py \
  --input $(ros2 pkg prefix arena_tag_map)/share/arena_tag_map/config/generate_map_request_example.yaml \
  --output-dir /tmp/generated_maps \
  --basename practice_field
```

Notes:
- Arena origin is set by `arena.origin_semantics` (`center` or `south_west_corner`).
- Object boxes are axis-aligned and use `south_west` as the south-west corner in map XY.
- Tag output includes both `apriltags.tags` and top-level `tags` for observer compatibility.
