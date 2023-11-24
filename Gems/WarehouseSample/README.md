# Warehouse Sample

This gem contains a sample warehouse scene, suitable for the ROS robotic simulations. 

The gem contains:
- `Warehouse.fbx` mesh asset
- `Terrain.fbx` mesh asset
- All necessary materials and textures.

Setting the scene - tips:
1. The `Warehouse` and `Terrain` assets should be located in the same coordinates. It is recommended to create the Terrain as a child of the Warehouse and set its translation and rotation to `0`.
2. The gem contains all the necessary materials. Names of materials correspond model materials in the fbx meshes.
3. Both fbx meshes can be used as PhysX colliders.
