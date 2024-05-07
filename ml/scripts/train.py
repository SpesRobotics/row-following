# # from ultralytics import YOLO

# # # Load a model
# # model = YOLO('yolov8n.yaml')  # build a new model from YAML
# # model = YOLO('yolov8n.pt')  # load a pretrained model (recommended for training)
# # model = YOLO('yolov8n.yaml').load('yolov8n.pt')  # build from YAML and transfer weights

# # # Train the model
# # results = model.train(data='data.yaml', epochs=100, imgsz=640)

# from ultralytics import YOLO

# # Load your dataset configuration
# dataset = '/home/milos/row-following/ros2_ws/Dataset/data.yaml'

# # Initialize the model
# model = YOLO(model='yolov8n.pt',  # Use 'yolov8n.pt' for YOLOv8n or another appropriate pre-trained model
#              data=dataset,
#              imgsz=640,  # Image size
#              batch_size=16,  # Batch size
#              epochs=50)  # Number of epochs

# # Train the model
# model.train()

from roboflow import Roboflow
from ultralytics import YOLO
from glob import glob
import yaml
import os
import argparse

# python3 train.py --key BoO3SVfFbG6tRGXoiLOM \
#                      --project row-following \
#                      --workspace faculty-of-tehnical-science \
#                      --destination milos@109.245.66.46:/home/milos/Desktop/TrainedModels \
#                      --steps train \
#                      --epochs 50 \
#                      --imgsz 640

# rf = Roboflow(api_key="BoO3SVfFbG6tRGXoiLOM")
# project = rf.workspace("faculty-of-tehnical-science").project("row-following")
# version = project.version(1)
# dataset = version.download("yolov8")

DATA_YAML_FILENAME = '/home/milos/row-following/ros2_ws/Dataset/data.yaml'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--key", help="Roboflow API key")
    parser.add_argument("--project", help="Roboflow project name")
    parser.add_argument("--workspace", help="Roboflow workspace name")
    parser.add_argument("--destination", help="Remote destination")
    parser.add_argument("--steps", default='download,train,upload', help="Steps to run")
    parser.add_argument("--epochs", default=50, help="Number of epochs")
    parser.add_argument("--imgsz", default=32*20, type=int, help="Image size")
    args = parser.parse_args()

    steps = args.steps.split(',')

    if 'download' in steps:
        rf = Roboflow(api_key=args.key)
        project = rf.workspace(args.workspace).project(args.project)

        # Generate and download the latest version of the dataset
        project.generate_version(
            {
                "augmentation": {},
                "preprocessing": {},
            }
        )
        latest_version = project.versions()[0].version
        latest_version_number = int(latest_version.split("/")[-1])
        project.version(latest_version_number).download("yolov8", location="dataset")

        # Fix the paths in data.yaml file
        data_dir = os.path.abspath(os.path.dirname(DATA_YAML_FILENAME))
        data_yaml = yaml.load(open(DATA_YAML_FILENAME), Loader=yaml.FullLoader)
        data_yaml["train"] = os.path.join(data_dir, "train", "images")
        data_yaml["val"] = os.path.join(data_dir, "valid", "images")
        data_yaml["test"] = os.path.join(data_dir, "test", "images")
        yaml.dump(data_yaml, open(DATA_YAML_FILENAME, "w"))

    if 'train' in steps:
        model = YOLO("yolov8n.pt")
        model.train(data=DATA_YAML_FILENAME, epochs=int(args.epochs), imgsz=int(args.imgsz))

    if 'upload' in steps:
        model_path = glob("**/best.pt", recursive=True)[-1]
        os.system(f"scp {model_path} {args.destination}")


if __name__ == "__main__":
    main()