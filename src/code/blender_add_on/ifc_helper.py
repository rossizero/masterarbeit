bl_info = {
    "name": "IFC Modeling Helper",
    "blender": (2, 80, 0),
    "category": "Object",
}

import bpy
import blenderbim.tool as tool
import time
import ifcopenshell.util.element

class IfcModelingHelper:
    last_selection = []
    ifc_file = tool.Ifc.get()
    GRID_IDENTIFICATOR = "grid"

    @classmethod
    def get_grid(cls, ifc_type):
        type_psets = ifcopenshell.util.element.get_psets(ifc_type)
        if type_psets is not None:
            for name in type_psets.keys():
                pset = type_psets[name]
                if cls.GRID_IDENTIFICATOR in pset.keys():
                    g = [round(n, 9) for n in pset[cls.GRID_IDENTIFICATOR]]
                    return g
        return None


    @classmethod
    def on_depsgraph_update(cls, scene, _):
        if bpy.context.selected_objects != cls.last_selection:
            cls.last_selection = bpy.context.selected_objects
            obj = bpy.context.active_object
            if obj.BIMObjectProperties.ifc_definition_id is not None:
                obj = cls.ifc_file.by_id(obj.BIMObjectProperties.ifc_definition_id)
                obj_type = ifcopenshell.util.element.get_type(obj)
                print(obj.is_a("IfcWall"))
                print(obj_type)
                type_psets = ifcopenshell.util.element.get_psets(obj_type)
                print(type_psets)
                obj_psets = ifcopenshell.util.element.get_psets(obj)
                print(obj_psets)
                grid = cls.get_grid(obj_type)
                if grid:
                    cls.set_snap_parameters(grid)
                print(grid)

            print("Updated", time.time())

        depsgraph = bpy.context.evaluated_depsgraph_get()

        for update in depsgraph.updates:
            obj = bpy.context.active_object
            if update.is_updated_transform:
                print(obj, " is_updated_transform")
            
            if update.is_updated_shading:
                print(obj, " is_updated_shading")

            if update.is_updated_geometry:
                print(obj, " is_updated_geometry")

    @classmethod
    def set_snap_parameters(cls, grid: []):
        for area in bpy.data.screens["Scripting"].areas:
            if area.type == 'VIEW_3D':
                for space in area.spaces:
                    if space.type == 'VIEW_3D':
                        space.overlay.grid_scale = grid[0]
                        break

        bpy.context.tool_settings.use_snap = True
        bpy.context.tool_settings.use_snap_grid_absolute = True
        bpy.context.scene.tool_settings.snap_elements = {'INCREMENT'}
        # https://blender.stackexchange.com/questions/154610/how-do-you-programatically-set-grid-scale
        # https://prosperocoder.com/posts/blender/blender-python-transformations/
        # https://docs.blender.org/api/current/bpy.types.DepsgraphUpdate.html

def register():
    # bpy.app.handlers.depsgraph_update_post.clear()
    bpy.app.handlers.depsgraph_update_post.append(IfcModelingHelper.on_depsgraph_update)

def unregister():
    bpy.app.handlers.depsgraph_update_post.remove(IfcModelingHelper.on_depsgraph_update)


# This allows you to run the script directly from Blender's Text editor
# to test the add-on without having to install it.
if __name__ == "__main__":
    register()