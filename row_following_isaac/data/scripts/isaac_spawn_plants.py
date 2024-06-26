import omni.isaac.core.utils.stage as stage_utils
import omni.isaac.core.utils.prims as prims_utils
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.prims import XFormPrim
from pxr import Gf

import math
import numpy as np
import random
import sys
sys.path.append('/home/milos/.local/lib/python3.10/site-packages')

import transforms3d

import omni.graph.core as og

# # Default values
# NUM_PLANTS = 40
# PLANT_SPACE = 0.1

# NUM_SUB_ROWS = 5
# SUB_ROW_SPACE = 0.07

# NUM_ROWS = 3
# ROW_SPACE = 0.5

#OBJ PATH
plant_usd_path = '/home/milos/row-following/row_following_isaac/data/USDs/persun_scaled.usda'
def spawn_plants(NUM_ROWS, ROW_SPACE, NUM_SUB_ROWS, SUB_ROW_SPACE, NUM_PLANTS, PLANT_SPACE):
    prims_utils.delete_prim("/World/persun")
    prims_utils.delete_prim("/World/spesbot")
    for i in range(NUM_ROWS):
        for j in range(NUM_SUB_ROWS):
            for k in range(NUM_PLANTS):
                x_position = k * PLANT_SPACE
                y_position = (SUB_ROW_SPACE * j) + (i * ((SUB_ROW_SPACE * (NUM_SUB_ROWS - 1)) + ROW_SPACE))
                plant_prim_path = f"/World/persun/persun_{i}_{j}_{k}"
                # if i == 0 and j == 0:
                stage_utils.add_reference_to_stage(plant_usd_path, plant_prim_path)

                parcel_prim = XFormPrim(
                    prim_path=plant_prim_path,
                    # name='test',
                    translation=[x_position, y_position, 0]
                )
                parcel_prim = prims_utils.get_prim_at_path(plant_prim_path)

                angle_degrees = random.uniform(0, 360)

                angle_radians = math.radians(angle_degrees)

                rot_mat = transforms3d.euler.euler2mat(0, 0, angle_radians)
                set_rotate(parcel_prim, rot_mat)

    robot_usd_path = '/home/milos/row-following/row_following_isaac/data/USDs/spesbot.usda'
    robot_prim_path = "/World/spesbot"
    stage_utils.add_reference_to_stage(robot_usd_path, robot_prim_path)

    parcel_prim = XFormPrim(
        prim_path=robot_prim_path,
        # name='test',
        translation=[0, (ROW_SPACE / 2) + ((NUM_SUB_ROWS - 1) * SUB_ROW_SPACE), 0]
    )
    parcel_prim = prims_utils.get_prim_at_path(robot_prim_path)

# angular = db.inputs.angular
# linear = db.inputs.angular

# prim_path = "/World/Camera"

# parcel_prim = prims_utils.get_prim_at_path(prim_path)

def set_translate(prim, new_loc):
    properties = prim.GetPropertyNames()
    if 'xformOp:translate' in properties:
        translate_attr = prim.GetAttribute('xformOp:translate')
        translate_attr.Set(new_loc)
    elif 'xformOp:translation' in properties:
        translation_attr = prim.GetAttribute('xformOp:translate')
        translation_attr.Set(new_loc)
    elif 'xformOp:transform' in properties:
        transform_attr = prim.GetAttribute('xformOp:transform')
        matrix = prim.GetAttribute('xformOp:transform').Get()
        matrix.SetTranslateOnly(new_loc)
        transform_attr.Set(matrix)
    else:
        xform = UsdGeom.Xformable(prim)
        xform_op = xform.AddXformOp(
            UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, '')
        xform_op.Set(new_loc)

def set_rotate(prim, rot_mat):
    properties = prim.GetPropertyNames()
    if 'xformOp:rotate' in properties:
        rotate_attr = prim.GetAttribute('xformOp:rotate')
        rotate_attr.Set(rot_mat)
    elif 'xformOp:transform' in properties:
        transform_attr = prim.GetAttribute('xformOp:transform')
        matrix = prim.GetAttribute('xformOp:transform').Get()
        matrix.SetRotateOnly(rot_mat.ExtractRotation())
        transform_attr.Set(matrix)
    elif 'xformOp:orient' in properties:
        orient_attr = prim.GetAttribute('xformOp:orient')
        q = transforms3d.quaternions.mat2quat(rot_mat)
        quad = Gf.Quatd(q[0], q[1], q[2], q[3])
        orient_attr.Set(quad)
    else:
        xform = UsdGeom.Xformable(prim)
        xform_op = xform.AddXformOp(
            UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble, '')
        q = transforms3d.quaternions.mat2quat(rot_mat)
        quad = Gf.Quatd(q[0], q[1], q[2], q[3])
        xform_op.Set(quad)

def setup(db: og.Database):
    pass

def cleanup(db: og.Database):
    print("Finished!")

def compute(db: og.Database):
    # test = db.inputs.test
    angular = db.inputs.angular
    linear = db.inputs.linear
    NUM_PLANTS = int(linear[0])
    PLANT_SPACE = angular[0]

    NUM_SUB_ROWS = int(linear[1])
    SUB_ROW_SPACE = angular[1]

    NUM_ROWS = int(linear[2])
    ROW_SPACE = angular[2]
    print(f"Angular: {angular}")
    print(f"Linear: {linear}")
    print(f"NUM_PLANTS: {NUM_PLANTS}")
    print(f"PLANT_SPACE: {PLANT_SPACE}")
    print(f"NUM_SUB_ROWS: {NUM_SUB_ROWS}")
    print(f"SUB_ROW_SPACE: {SUB_ROW_SPACE}")
    print(f"NUM_ROWS: {NUM_ROWS}")
    print(f"ROW_SPACE: {ROW_SPACE}")

    prims_utils.delete_prim("/World/persun")
    # prims_utils.delete_prim("/World/spesbot")
    for i in range(NUM_ROWS):
        for j in range(NUM_SUB_ROWS):
            for k in range(NUM_PLANTS):
                x_position = k * PLANT_SPACE
                y_position = (SUB_ROW_SPACE * j) + (i * ((SUB_ROW_SPACE * (NUM_SUB_ROWS - 1)) + ROW_SPACE))
                plant_prim_path = f"/World/persun/persun_{i}_{j}_{k}"
                # if i == 0 and j == 0:
                stage_utils.add_reference_to_stage(plant_usd_path, plant_prim_path)

                parcel_prim = XFormPrim(
                    prim_path=plant_prim_path,
                    # name='test',
                    translation=[x_position, y_position, 0]
                )
                parcel_prim = prims_utils.get_prim_at_path(plant_prim_path)

                angle_degrees = random.uniform(0, 360)

                angle_radians = math.radians(angle_degrees)

                rot_mat = transforms3d.euler.euler2mat(0, 0, angle_radians)
                set_rotate(parcel_prim, rot_mat)

    # robot_usd_path = '/home/milos/row-following/row_following_isaac/data/USDs/spesbot.usda'
    # robot_prim_path = "/World/spesbot"
    # stage_utils.add_reference_to_stage(robot_usd_path, robot_prim_path)

    # parcel_prim = XFormPrim(
    #     prim_path=robot_prim_path,
    #     # name='test',
    #     translation=[0, (ROW_SPACE / 2) + ((NUM_SUB_ROWS - 1) * SUB_ROW_SPACE), 0]
    # )
    # parcel_prim = prims_utils.get_prim_at_path(robot_prim_path)

    # spawn_plants(NUM_ROWS, ROW_SPACE, NUM_SUB_ROWS, SUB_ROW_SPACE, NUM_PLANTS, PLANT_SPACE)
    return True
    # print(f"Angular {angular}")
    # print(f"Linear {linear}")
    # print(f"Angular[2] {angular[2]}")
    # roll_angle_degrees = random.uniform(70, 80)

    # roll_angle_degrees = random.uniform(30, 45)
    # pitch_angle_degrees = random.uniform(-10, 10)
    # yaw_angle_degrees = random.uniform(-85, -95)

    # roll_angle_radians = math.radians(roll_angle_degrees)
    # pitch_angle_radians = math.radians(pitch_angle_degrees)
    # yaw_angle_radians = math.radians(yaw_angle_degrees)

    # rot_mat = transforms3d.euler.euler2mat(roll_angle_radians, pitch_angle_radians, yaw_angle_radians)
    # set_rotate(parcel_prim, rot_mat)

    # # translate = Gf.Vec3d(random.uniform(-0.2, 1.4), random.uniform(0.4, 2.2), random.uniform(0.49, 0.51))
    # translate = Gf.Vec3d(random.uniform(-0.5, 3.5), random.uniform(-0.2, 2.2), random.uniform(0.49, 0.51))

    # set_translate(parcel_prim, translate)
    # return True


# #OBJ PATH
# plant_usd_path = '/home/milos/row-following/row_following_isaac/data/USDs/persun_scaled.usda'
# def spawn_plants(NUM_ROWS, ROW_SPACE, NUM_SUB_ROWS, SUB_ROW_SPACE, NUM_PLANTS, PLANT_SPACE):
#     prims_utils.delete_prim("/World/persun")
#     prims_utils.delete_prim("/World/spesbot")
#     for i in range(NUM_ROWS):
#         for j in range(NUM_SUB_ROWS):
#             for k in range(NUM_PLANTS):
#                 x_position = k * PLANT_SPACE
#                 y_position = (SUB_ROW_SPACE * j) + (i * ((SUB_ROW_SPACE * (NUM_SUB_ROWS - 1)) + ROW_SPACE))
#                 plant_prim_path = f"/World/persun/persun_{i}_{j}_{k}"
#                 # if i == 0 and j == 0:
#                 stage_utils.add_reference_to_stage(plant_usd_path, plant_prim_path)

#                 parcel_prim = XFormPrim(
#                     prim_path=plant_prim_path,
#                     # name='test',
#                     translation=[x_position, y_position, 0]
#                 )
#                 parcel_prim = prims_utils.get_prim_at_path(plant_prim_path)

#                 angle_degrees = random.uniform(0, 360)

#                 angle_radians = math.radians(angle_degrees)

#                 rot_mat = transforms3d.euler.euler2mat(0, 0, angle_radians)
#                 set_rotate(parcel_prim, rot_mat)

#     robot_usd_path = '/home/milos/row-following/row_following_isaac/data/USDs/spesbot.usda'
#     robot_prim_path = "/World/spesbot"
#     stage_utils.add_reference_to_stage(robot_usd_path, robot_prim_path)

#     parcel_prim = XFormPrim(
#         prim_path=robot_prim_path,
#         # name='test',
#         translation=[0, (ROW_SPACE / 2) + ((NUM_SUB_ROWS - 1) * SUB_ROW_SPACE), 0]
#     )
#     parcel_prim = prims_utils.get_prim_at_path(robot_prim_path)
