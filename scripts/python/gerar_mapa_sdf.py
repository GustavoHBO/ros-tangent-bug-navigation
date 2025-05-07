import random
import xml.etree.ElementTree as ET
 
def boxes_overlap(box1, box2):
    # Check for rectangle overlap in 2D
    x1, y1, w1, h1 = box1
    x2, y2, w2, h2 = box2
    return not (x1 + w1/2 <= x2 - w2/2 or x1 - w1/2 >= x2 + w2/2 or
                y1 + h1/2 <= y2 - h2/2 or y1 - h1/2 >= y2 + h2/2)
 
def generate_world_with_boxes(
    num_boxes,
    map_size=(20, 20),
    box_size_range=(0.5, 2.0),
    start_pos=(1, 1),
    goal_pos=(18, 18),
    clear_radius=2.0,
    wall_thickness=0.2,
    output_file="custom_world.sdf"
):
    placed_boxes = []
 
    def is_valid_position(x, y, sx, sy):
        # Check clear zones and existing boxes
        for cx, cy in [start_pos, goal_pos]:
            if ((x - cx)**2 + (y - cy)**2)**0.5 < clear_radius:
                return False
        for bx, by, bw, bh in placed_boxes:
            if boxes_overlap((x, y, sx, sy), (bx, by, bw, bh)):
                return False
        return True
 
    sdf = ET.Element("sdf", version="1.6")
    world = ET.SubElement(sdf, "world", name="default")
 
    # Physics
    physics = ET.SubElement(world, "physics", type="ode")
    ET.SubElement(physics, "real_time_update_rate").text = "1000"
    ET.SubElement(physics, "gravity").text = "0 0 -9.81"
 
    # Ground plane with texture
    ground = ET.SubElement(world, "model", name="ground_plane")
    ET.SubElement(ground, "static").text = "true"
    link = ET.SubElement(ground, "link", name="ground_link")
 
    for tag in ["collision", "visual"]:
        elem = ET.SubElement(link, tag, name=tag)
        geom = ET.SubElement(elem, "geometry")
        plane = ET.SubElement(geom, "plane")
        ET.SubElement(plane, "normal").text = "0 0 1"
        ET.SubElement(plane, "size").text = f"{map_size[0]} {map_size[1]}"
        if tag == "visual":
            mat = ET.SubElement(elem, "material")
            script = ET.SubElement(mat, "script")
            ET.SubElement(script, "uri").text = "file://media/materials/scripts/gazebo.material"
            ET.SubElement(script, "name").text = "Gazebo/Grass"
 
    # Walls
    def add_wall(name, x, y, x_size, y_size):
        wall = ET.SubElement(world, "model", name=name)
        ET.SubElement(wall, "static").text = "true"
        ET.SubElement(wall, "pose").text = f"{x} {y} 1.0 0 0 0"
        link = ET.SubElement(wall, "link", name="link")
        for tag in ["collision", "visual"]:
            elem = ET.SubElement(link, tag, name=tag)
            geom = ET.SubElement(elem, "geometry")
            box = ET.SubElement(geom, "box")
            ET.SubElement(box, "size").text = f"{x_size} {y_size} 2.0"
 
    w, h = map_size
    t = wall_thickness
    add_wall("wall_north", 0, h/2 + t/2, w, t)
    add_wall("wall_south", 0, -h/2 - t/2, w, t)
    add_wall("wall_east",  w/2 + t/2, 0, t, h)
    add_wall("wall_west", -w/2 - t/2, 0, t, h)
 
    # Light
    include = ET.SubElement(world, "include")
    ET.SubElement(include, "uri").text = "model://sun"
 
    # Boxes
    for i in range(num_boxes):
        size_x = round(random.uniform(*box_size_range), 2)
        size_y = round(random.uniform(*box_size_range), 2)
        half_x, half_y = size_x / 2, size_y / 2
 
        for _ in range(100):  # Try up to 100 times to place a non-overlapping box
            x = round(random.uniform(-w/2 + half_x, w/2 - half_x), 2)
            y = round(random.uniform(-h/2 + half_y, h/2 - half_y), 2)
            if is_valid_position(x, y, size_x, size_y):
                placed_boxes.append((x, y, size_x, size_y))
                break
        else:
            print(f"[!] Could not place box {i} without overlapping. Skipping.")
            continue
 
        box = ET.SubElement(world, "model", name=f"box_{i}")
        ET.SubElement(box, "pose").text = f"{x} {y} 0.25 0 0 0"
        ET.SubElement(box, "static").text = "false"
        link = ET.SubElement(box, "link", name="link")
 
        for tag in ["collision", "visual"]:
            elem = ET.SubElement(link, tag, name=tag)
            geom = ET.SubElement(elem, "geometry")
            box_elem = ET.SubElement(geom, "box")
            ET.SubElement(box_elem, "size").text = f"{size_x} {size_y} 0.5"
            if tag == "visual":
                mat = ET.SubElement(elem, "material")
                script = ET.SubElement(mat, "script")
                ET.SubElement(script, "uri").text = "file://media/materials/scripts/gazebo.material"
                ET.SubElement(script, "name").text = "Gazebo/Bricks"
 
    # Write SDF to file
    tree = ET.ElementTree(sdf)
    tree.write(output_file, encoding="utf-8", xml_declaration=True)
    print(f"[✓] World file saved to '{output_file}'")
 
# Example usage
if __name__ == "__main__":
    generate_world_with_boxes(
        num_boxes=30,
        map_size=(20, 20),
        start_pos=(1, 1),
        goal_pos=(18, 18),
        clear_radius=2.0,
        output_file="generated_world.sdf"
    )