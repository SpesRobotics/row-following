import omni.isaac.core.utils.stage as stage_utils
import omni.isaac.core.utils.prims as prims_utils
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.prims import XFormPrim

NUM_PLANTS = 12
PLANT_SPACE = 1.5
NUM_ROWS = 5
ROW_SPACE = 2

# GroundPlane(prim_path="/World/groundPlane", size=10, color=np.array([0.5, 0.5, 0.5]))

#OBJ PATH
usd_path = '/home/milos/row-following/Simulation/persun_isaac.usd'

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
