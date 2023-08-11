import os
from PIL import Image

source_dir = 'images/depth'
dest_dir = 'images/depth2'

os.makedirs(dest_dir, exist_ok=True)

def resize_and_crop(image_path, output_path, size):
    original_image = Image.open(image_path)
    original_width, original_height = original_image.size
    original_ratio = original_width / original_height
    target_ratio = size[0] / size[1]

    if original_ratio > target_ratio:
        wb_resize_width = size[0]
        wb_resize_height = round(original_height * size[0] / original_width)
        intermediate_image = original_image.resize((wb_resize_width, wb_resize_height))
    else:
        hb_resize_width = round(original_width * size[1] / original_height)
        hb_resize_height = size[1]
        intermediate_image = original_image.resize((hb_resize_width, hb_resize_height))

    crop_x = (intermediate_image.size[0] - size[0]) / 2
    crop_y = (intermediate_image.size[1] - size[1]) / 2
    final_image = intermediate_image.crop((crop_x, crop_y, crop_x + size[0], crop_y + size[1]))
    final_image.save(output_path)

for filename in os.listdir(source_dir):
    if filename.endswith('.jpg'):
        image_path = os.path.join(source_dir, filename)
        output_path = os.path.join(dest_dir, filename)
        resize_and_crop(image_path, output_path, (640, 480))
