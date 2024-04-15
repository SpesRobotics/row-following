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

NUM_PLANTS = 20
PLANT_SPACE = 0.1
NUM_ROWS = 5
ROW_SPACE = 0.4

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

# GroundPlane(prim_path="/World/groundPlane", size=10, color=np.array([0.5, 0.5, 0.5]))

#OBJ PATH
usd_path = '/home/milos/row-following/Simulation/persun_scaled.usda'

for i in range(NUM_ROWS):
    for j in range(NUM_PLANTS):
        x_position = j * PLANT_SPACE
        y_position = i * ROW_SPACE
        prim_path = f"/World/persun_{i}_{j}"
        # if i == 0 and j == 0:
        stage_utils.add_reference_to_stage(usd_path, prim_path)

        parcel_prim = XFormPrim(
            prim_path=prim_path,
            # name='test',
            translation=[x_position, y_position, 0]
        )
        parcel_prim = prims_utils.get_prim_at_path(prim_path)

        # angle_degrees = random.uniform(0, 360)

        # angle_radians = math.radians(angle_degrees)

        # rot_mat = transforms3d.euler.euler2mat(0, 0, angle_radians)
        # set_rotate(parcel_prim, rot_mat)


# prim_path = '/World/persunovo'

# stage_utils.add_reference_to_stage(usd_path, prim_path)

# parcel_prim = XFormPrim(
#     prim_path=prim_path,
#     # name='tesst',
#     translation=[0, 0, 0]
# )
# parcel_prim = prims_utils.get_prim_at_path(prim_path)

# parcel_prim.CreateAttribute("semantic:Semantics_0CQz:params:semanticType", Sdf.ValueTypeNames.String)
# parcel_prim.CreateAttribute("semantic:Semantics_0CQz:params:semanticData", Sdf.ValueTypeNames.String)
# parcel_prim.GetAttribute("semantic:Semantics_0CQz:params:semanticType").Set("class")
# parcel_prim.GetAttribute("semantic:Semantics_0CQz:params:semanticData").Set("plant")
