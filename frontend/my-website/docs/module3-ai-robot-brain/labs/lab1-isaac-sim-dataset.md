# Lab 1: Generating a Dataset using Isaac Sim

## Objective

In this lab, you will learn how to use Isaac Sim to generate a synthetic dataset for training a computer vision model. You will create a simple scene with various objects, configure sensors to capture RGB and depth data, set up ground truth annotations, and run a data generation script to create a dataset suitable for object detection or segmentation tasks.

## Prerequisites

- NVIDIA GPU with RTX capabilities (for optimal performance)
- Isaac Sim installed and properly configured
- Omniverse Nucleus server (if using multi-user setup)
- Python 3.8+ with pip
- Basic understanding of computer vision concepts
- Familiarity with USD (Universal Scene Description) format

## Setup

### 1. Install Required Python Packages

Before starting, ensure you have the necessary Python packages installed:

```bash
# Create a virtual environment (recommended)
python -m venv isaac_sim_dataset_env
source isaac_sim_dataset_env/bin/activate  # On Windows: isaac_sim_dataset_env\Scripts\activate

# Install Isaac Sim Python modules (these should be available in Isaac Sim's Python environment)
# If running outside Isaac Sim, you may need to install additional packages
pip install numpy opencv-python pillow
```

### 2. Verify Isaac Sim Installation

Launch Isaac Sim to ensure it's properly installed:

```bash
# If Isaac Sim is installed as a standalone application
./isaac-sim/isaac-sim.sh

# Or if using Isaac ROS Docker
docker run --gpus all -it --rm --network=host -v ~/isaac-sim-cache:/isaac-sim-cache isaac_ros:galactic
```

## Lab Steps

### 1. Create a New Scene in Isaac Sim

1. Launch Isaac Sim
2. Create a new stage (File → New Stage)
3. Set up a simple environment:
   - Add a ground plane: Create → Ground Plane
   - Add a dome light: Create → Dome Light (for realistic lighting)
   - Add a distant light: Create → Distant Light (for additional illumination)

### 2. Add Objects to the Scene

1. From the Isaac Sim asset library, add various objects to your scene:
   - Go to Window → Content Browser
   - Navigate to `Isaac/Props/Blocks` or similar folders
   - Drag and drop several different colored blocks into the scene
   - Vary their positions, rotations, and scales for diversity

2. Example objects to include:
   - Colored cubes (red, blue, green, yellow)
   - Spheres of different sizes
   - Cylinders
   - Simple articulated objects

### 3. Configure Camera and Sensors

1. Add a camera to the scene:
   - Create → Camera
   - Position the camera to view the objects on the ground plane
   - Set the camera resolution (e.g., 640x480 or 1280x720)

2. Configure the camera for RGB and depth capture:
   - Select the camera prim
   - In the Property window, expand `Isaac Sensors`
   - Add a `RtxSensor` component
   - Configure the sensor to capture both RGB and depth data

### 4. Set Up Ground Truth Annotations

1. Enable semantic segmentation:
   - Select each object in your scene
   - In the Property window, assign a semantic label (e.g., "red_cube", "blue_sphere")
   - This will allow Isaac Sim to generate semantic segmentation masks

2. Configure annotation settings:
   - Go to Window → Isaac Examples → Synthetic Data
   - This provides tools for configuring different types of ground truth data

### 5. Create a Python Script for Data Generation

Create a Python script that will automate the data generation process. Save this as `generate_dataset.py` in your project directory:

```python
import omni
import omni.kit.commands
from pxr import Gf, UsdGeom
import numpy as np
import cv2
import os
from PIL import Image
import random

# Isaac Sim imports
from omni.isaac.synthetic_utils import SyntheticDataHelper
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.rotations import euler_angles_to_quat

class DatasetGenerator:
    def __init__(self, output_dir="dataset", num_frames=1000):
        self.output_dir = output_dir
        self.num_frames = num_frames
        self.world = None
        self.sd_helper = None

        # Create output directories
        os.makedirs(os.path.join(output_dir, "rgb"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "depth"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "segmentation"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "annotations"), exist_ok=True)

    def setup_scene(self):
        """Set up the scene with objects"""
        # Initialize the world
        self.world = World(stage_units_in_meters=1.0)

        # Add ground plane
        omni.kit.commands.execute(
            "CreateGroundPlaneCommand",
            plane_path="/World/GroundPlane",
            axis="Z"
        )

        # Add dome light
        omni.kit.commands.execute(
            "CreateDomeLightCommand",
            light_path="/World/DomeLight",
            color=Gf.Vec3f(0.5, 0.5, 0.5)
        )

        # Add camera
        camera_path = "/World/Camera"
        omni.kit.commands.execute(
            "CreateCameraAndAttachToStage",
            camera_path=camera_path
        )

        # Position camera
        camera_prim = get_prim_at_path(camera_path)
        camera_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(2.0, 0.0, 2.0))
        camera_prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(-45, 0, 0))

        # Add objects to scene
        self.add_objects_to_scene()

        # Initialize synthetic data helper
        self.sd_helper = SyntheticDataHelper()
        self.sd_helper.initialize(sensor_names=["Camera", "Camera_helper"])

    def add_objects_to_scene(self):
        """Add random objects to the scene"""
        # Define object types and colors
        objects = [
            ("cube", "red", (1.0, 0.0, 0.0)),
            ("cube", "blue", (0.0, 0.0, 1.0)),
            ("sphere", "green", (0.0, 1.0, 0.0)),
            ("cylinder", "yellow", (1.0, 1.0, 0.0))
        ]

        # Add 10 random objects
        for i in range(10):
            obj_type, color, rgb_color = random.choice(objects)

            # Random position
            x = random.uniform(-1.0, 1.0)
            y = random.uniform(-1.0, 1.0)
            z = 0.5  # Place on ground

            # Random rotation
            rot = (random.uniform(0, 360), random.uniform(0, 360), random.uniform(0, 360))

            # Random scale
            scale = random.uniform(0.3, 0.7)

            # Create object based on type
            if obj_type == "cube":
                omni.kit.commands.execute(
                    "CreateMeshPrimWithDefaultXform",
                    prim_type="Cube",
                    prim_path=f"/World/Object_{i}",
                    position=(x, y, z),
                    orientation=euler_angles_to_quat(rot, degrees=True),
                    scale=(scale, scale, scale)
                )
            elif obj_type == "sphere":
                omni.kit.commands.execute(
                    "CreateMeshPrimWithDefaultXform",
                    prim_type="Sphere",
                    prim_path=f"/World/Object_{i}",
                    position=(x, y, z),
                    orientation=euler_angles_to_quat(rot, degrees=True),
                    scale=(scale, scale, scale)
                )
            elif obj_type == "cylinder":
                omni.kit.commands.execute(
                    "CreateMeshPrimWithDefaultXform",
                    prim_type="Cylinder",
                    prim_path=f"/World/Object_{i}",
                    position=(x, y, z),
                    orientation=euler_angles_to_quat(rot, degrees=True),
                    scale=(scale, scale, scale)
                )

            # Apply color
            omni.kit.commands.execute(
                "ChangePropertyCommand",
                prop_path=f"/World/Object_{i}.primvars:displayColor",
                prop_type=omni.usd.get_prop_type_from_python_type(Gf.Vec3f),
                value=Gf.Vec3f(*rgb_color)
            )

            # Add semantic label
            omni.kit.commands.execute(
                "AddSemanticLabel",
                prim_path=f"/World/Object_{i}",
                label=f"{color}_{obj_type}",
                type_label="default"
            )

    def generate_dataset(self):
        """Generate the synthetic dataset"""
        print(f"Generating {self.num_frames} frames of synthetic data...")

        for frame_idx in range(self.num_frames):
            # Randomize scene slightly
            self.randomize_scene()

            # Step the physics simulation
            self.world.step(render=True)

            # Capture synthetic data
            try:
                # Get RGB data
                rgb_data = self.sd_helper.get_rgb_data(sensor_name="Camera")
                if rgb_data is not None:
                    rgb_image = Image.fromarray(rgb_data, mode="RGB")
                    rgb_image.save(os.path.join(self.output_dir, "rgb", f"rgb_{frame_idx:06d}.png"))

                # Get depth data
                depth_data = self.sd_helper.get_depth_data(sensor_name="Camera")
                if depth_data is not None:
                    # Normalize depth for visualization (optional)
                    depth_normalized = ((depth_data - depth_data.min()) / (depth_data.max() - depth_data.min()) * 255).astype(np.uint8)
                    depth_image = Image.fromarray(depth_normalized, mode="L")
                    depth_image.save(os.path.join(self.output_dir, "depth", f"depth_{frame_idx:06d}.png"))

                # Get segmentation data
                segmentation_data = self.sd_helper.get_semantic_segmentation(sensor_name="Camera")
                if segmentation_data is not None:
                    # Convert to appropriate format for segmentation
                    seg_image = Image.fromarray(segmentation_data.astype(np.uint8), mode="L")
                    seg_image.save(os.path.join(self.output_dir, "segmentation", f"seg_{frame_idx:06d}.png"))

                # Log progress
                if frame_idx % 100 == 0:
                    print(f"Generated {frame_idx}/{self.num_frames} frames")

            except Exception as e:
                print(f"Error capturing frame {frame_idx}: {e}")
                continue

        print(f"Dataset generation complete! Data saved to {self.output_dir}")

    def randomize_scene(self):
        """Randomize the scene for variation"""
        # Move camera to random position around the objects
        angle = random.uniform(0, 2 * 3.14159)
        radius = random.uniform(1.5, 3.0)
        height = random.uniform(1.0, 2.5)

        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        z = height

        camera_prim = get_prim_at_path("/World/Camera")
        camera_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(x, y, z))

        # Randomly move some objects slightly
        for i in range(5):  # Move 5 random objects
            obj_idx = random.randint(0, 9)
            current_pos_attr = get_prim_at_path(f"/World/Object_{obj_idx}").GetAttribute("xformOp:translate")
            current_pos = current_pos_attr.Get()

            new_x = current_pos[0] + random.uniform(-0.1, 0.1)
            new_y = current_pos[1] + random.uniform(-0.1, 0.1)
            new_z = current_pos[2]  # Keep z the same

            get_prim_at_path(f"/World/Object_{obj_idx}").GetAttribute("xformOp:translate").Set(Gf.Vec3d(new_x, new_y, new_z))

def main():
    # Initialize Isaac Sim world
    omni.kit.commands.execute("IsaacSimToggleGrid", visible=False)
    omni.kit.commands.execute("IsaacSimToggleRulers", visible=False)

    # Create dataset generator
    generator = DatasetGenerator(output_dir="isaac_sim_dataset", num_frames=500)  # Generate 500 frames

    # Set up the scene
    generator.setup_scene()

    # Generate the dataset
    generator.generate_dataset()

    # Cleanup
    if generator.world:
        generator.world.clear()

if __name__ == "__main__":
    main()
```

### 6. Run the Data Generation Script

1. Save the script in your Isaac Sim project directory
2. Run the script from within Isaac Sim's Python environment:

```bash
# In Isaac Sim, open the Script Editor (Window → Extensions → Script Editor)
# Or run from command line if Isaac Sim Python is in your path
python generate_dataset.py
```

### 7. Verify the Generated Dataset

After the script completes, verify that the dataset has been generated correctly:

1. Check the output directory structure:
   ```
   dataset/
   ├── rgb/
   │   ├── rgb_000000.png
   │   ├── rgb_000001.png
   │   └── ...
   ├── depth/
   │   ├── depth_000000.png
   │   ├── depth_000001.png
   │   └── ...
   ├── segmentation/
   │   ├── seg_000000.png
   │   ├── seg_000001.png
   │   └── ...
   └── annotations/
   ```

2. Verify the content of a few sample images to ensure they contain the expected data.

### 8. Format the Dataset for Training

To use the generated dataset for training, you may need to convert it to a standard format like COCO or YOLO:

```python
import json
import os
from PIL import Image

def convert_to_coco_format(dataset_dir, output_file):
    """Convert the generated dataset to COCO format"""
    coco_format = {
        "info": {
            "description": "Synthetic Dataset Generated with Isaac Sim",
            "version": "1.0",
            "year": 2024
        },
        "images": [],
        "annotations": [],
        "categories": [
            {"id": 1, "name": "red_cube"},
            {"id": 2, "name": "blue_cube"},
            {"id": 3, "name": "green_sphere"},
            {"id": 4, "name": "yellow_cylinder"}
        ]
    }

    # Add images
    rgb_dir = os.path.join(dataset_dir, "rgb")
    for i, filename in enumerate(sorted(os.listdir(rgb_dir))):
        img_path = os.path.join(rgb_dir, filename)
        img = Image.open(img_path)
        width, height = img.size

        coco_format["images"].append({
            "id": i,
            "file_name": filename,
            "width": width,
            "height": height
        })

    # Save the COCO format dataset
    with open(output_file, 'w') as f:
        json.dump(coco_format, f)

# Convert the dataset
convert_to_coco_format("isaac_sim_dataset", "dataset_coco_format.json")
```

## Troubleshooting

- **Empty Images**: Ensure the camera is properly positioned and objects are within the camera's view frustum
- **Performance Issues**: Reduce scene complexity or frame count for initial testing
- **Memory Issues**: Process frames in smaller batches
- **Missing Annotations**: Verify semantic labels are properly assigned to objects

## Conclusion

You have successfully generated a synthetic dataset using Isaac Sim. This dataset can be used to train computer vision models for tasks such as object detection, segmentation, or pose estimation. The synthetic data provides ground truth annotations that are often difficult or expensive to obtain in real-world datasets.

## Further Exploration

- Experiment with different lighting conditions and camera angles
- Add more complex objects and scenes
- Implement domain randomization techniques
- Explore Isaac Sim's built-in dataset generation tools
- Integrate with popular training frameworks like PyTorch or TensorFlow