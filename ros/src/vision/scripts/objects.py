import os
import json

VISION_PATH = os.path.join(os.path.dirname(__file__), '..')
OBJECT_JSON = os.path.join(VISION_PATH, 'objects.json')

objects = {}

with open(OBJECT_JSON, 'r') as f:
    objects = json.load(f)
    
def get_object_position(class_name, position, img_size):
    height_px = objects[class_name]['height_px']
    width_px = objects[class_name]['width_px']
    real_height = objects[class_name]['real_height']
    real_width = objects[class_name]['real_width']
    real_distance = objects[class_name]['real_distance']
    
    img_height, img_width = img_size
    ymin, xmin, ymax, xmax = position
    delta_x = xmax - xmin
    delta_y = ymax - ymin

    # approx distance to object
    dist_meters = distance / (real_height / delta_y)
    
    # centre pixels
    origin_x = img_width / 2 
    origin_y = img_height / 2
    unit_scale = real_width / width_px
    center_x = ((xmin + delta_x) - origin_x) * unit_scale
    center_y = ((ymin + delta_y) - origin_y) * unit_scale

    return (center_x, center_y, dist_meters)
    
    



    

