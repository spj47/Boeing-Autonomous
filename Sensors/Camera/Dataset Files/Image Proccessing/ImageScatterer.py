import os
import shutil
import random

# Paths
raw_images_dir =  "Sensors/Camera/Dataset Files/Image Proccessing/RawImages"
labels_dir =  "Sensors/Camera/Dataset Files/Image Proccessing/Labels"

train_images_dir =  "Sensors/Camera/Dataset Files/Dataset/images/train"
val_images_dir =  "Sensors/Camera/Dataset Files/Dataset/images/val"
train_labels_dir =  "Sensors/Camera/Dataset Files/Dataset/Labels/train"
val_labels_dir =  "Sensors/Camera/Dataset Files/Dataset/Labels/val"

# Create target directories if they don't exist
for d in [train_images_dir, val_images_dir, train_labels_dir, val_labels_dir]:
    os.makedirs(d, exist_ok=True)

# Get all image files
images = [f for f in os.listdir(raw_images_dir) if os.path.isfile(os.path.join(raw_images_dir, f))]

# Shuffle for randomness
random.shuffle(images)

# Split index
split_idx = int(len(images) * 0.75)
train_images = images[:split_idx]
val_images = images[split_idx:]

# Function to move files and match labels by replacing extension
def move_files(file_list, src_img_dir, src_label_dir, dst_img_dir, dst_label_dir):
    for img_file in file_list:
        # Copy image
        shutil.copy(os.path.join(src_img_dir, img_file), os.path.join(dst_img_dir, img_file))
        
        # Find corresponding label by changing extension to .txt
        label_file = os.path.splitext(img_file)[0] + ".txt"
        src_label_path = os.path.join(src_label_dir, label_file)
        if os.path.exists(src_label_path):
            shutil.copy(src_label_path, os.path.join(dst_label_dir, label_file))
        else:
            print(f"Warning: Label not found for {img_file}")

# Move training files
move_files(train_images, raw_images_dir, labels_dir, train_images_dir, train_labels_dir)

# Move validation files
move_files(val_images, raw_images_dir, labels_dir, val_images_dir, val_labels_dir)

print(f"Split complete! {len(train_images)} training and {len(val_images)} validation images.")