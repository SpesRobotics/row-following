import omni.isaac.core.utils.stage as stage_utils
import omni.isaac.core.utils.prims as prims_utils
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.prims import XFormPrim
from pxr import Gf

import math
import random

import sys
sys.path.append('/home/milos/.local/lib/python3.10/site-packages')

import transforms3d

prim_path = "/World/Camera"

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

parcel_prim = prims_utils.get_prim_at_path(prim_path)


# roll_angle_degrees = random.uniform(70, 80)
# pitch_angle_degrees = random.uniform(-10, 10)
# yaw_angle_degrees = random.uniform(0, 360)

# roll_angle_radians = math.radians(roll_angle_degrees)
# pitch_angle_radians = math.radians(pitch_angle_degrees)
# yaw_angle_radians = math.radians(yaw_angle_degrees)

# rot_mat = transforms3d.euler.euler2mat(roll_angle_radians, pitch_angle_radians, yaw_angle_radians)
# set_rotate(parcel_prim, rot_mat)

# translate = Gf.Vec3d(random.uniform(-0.1, 2.1), random.uniform(0.3, 2.2), random.uniform(0.39, 0.41))
# set_translate(parcel_prim, translate)

roll_angle_degrees = random.uniform(30, 45)
pitch_angle_degrees = random.uniform(-10, 10)
yaw_angle_degrees = random.uniform(-85, -95)

roll_angle_radians = math.radians(roll_angle_degrees)
pitch_angle_radians = math.radians(pitch_angle_degrees)
yaw_angle_radians = math.radians(yaw_angle_degrees)

rot_mat = transforms3d.euler.euler2mat(roll_angle_radians, pitch_angle_radians, yaw_angle_radians)
set_rotate(parcel_prim, rot_mat)

# translate = Gf.Vec3d(random.uniform(-0.2, 1.4), random.uniform(0.6, 2.2), random.uniform(0.39, 0.41))
translate = Gf.Vec3d(random.uniform(-0.2, 1.4), random.uniform(0.4, 2.2), random.uniform(0.49, 0.51))

# translate = Gf.Vec3d(random.uniform(-0.5, 3.5), random.uniform(-0.2, 2.2), random.uniform(0.49, 0.51))

set_translate(parcel_prim, translate)

#RANDOM POSITIONS(20x12)

# from omni.isaac.dynamic_control import _dynamic_control
# from pxr import Gf

# # Acquire dynamic control interface
# dc = _dynamic_control.acquire_dynamic_control_interface()

# # Get the prim you want to move
# prim_path = '/World/persun_0_0'
# prim = dc.get_rigid_body(prim_path)

# # Set the new location
# new_location = Gf.Vec3f(1.0, 2.0, 3.0)  # Replace with your desired location

# # Set the new rotation
# new_rotation = Gf.Rotation(Gf.Vec3d(1, 0, 0), 0)  # Replace with your desired rotation

# # Create a new transform
# new_transform = Gf.Matrix4d(new_rotation.GetMatrix(), new_location)

# # Apply the new transform
# dc.set_rigid_body_pose(prim, new_transform)
