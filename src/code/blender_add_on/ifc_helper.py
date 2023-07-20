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
    reset_settings = []
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
        # selection changed?
        if bpy.context.selected_objects != cls.last_selection:
            cls.last_selection = bpy.context.selected_objects
            obj = bpy.context.active_object

            # set global grid to the one that's defined by the active ifcwall
            if obj.BIMObjectProperties.ifc_definition_id is not None:
                # save grid settings before changing them so we're able to reset them on selection change
                cls.save_settings(scene)

                cls.ifc_file = tool.Ifc.get()
                obj = cls.ifc_file.by_id(obj.BIMObjectProperties.ifc_definition_id)
                if obj is not None:
                    if obj.is_a("IfcWindow") or obj.is_a("IfcDoor"):
                        # https://ifc43-docs.standards.buildingsmart.org/IFC/RELEASE/IFC4x3/HTML/lexical/IfcDoor.htm
                        # does this door/window fill a void
                        fills = obj.FillsVoids  # [0:1]
                        if len(fills) > 0:
                            ifc_rel_fills_element = fills[0]
                            # which IfcOpeningElements are filled by this IfcRelFillElement
                            ifc_rel_opening_element = ifc_rel_fills_element.RelatingOpeningElement
                            void_element = ifc_rel_opening_element.VoidsElements[0]   # [1:1]
                            building_element = void_element.RelatingBuildingElement
                            obj = building_element

                    if obj.is_a("IfcWall"):
                        obj_type = ifcopenshell.util.element.get_type(obj)
                        grid = cls.get_grid(obj_type)
                        if grid:
                            cls.set_snap_parameters(scene, grid)
                            print(grid)
            else:
                # reset grid settings
                cls.reset_settings(scene)


            print("Selection changed", time.time())

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
    def save_settings(cls, scene):
        grid_scale = None
        for area in bpy.data.screens["Scripting"].areas:
            if area.type == 'VIEW_3D':
                for space in area.spaces:
                    if space.type == 'VIEW_3D':
                        grid_scale = space.overlay.grid_scale
                        break

        cls.reset_settings = [
            grid_scale,
            scene.tool_settings.snap_elements,
            scene.tool_settings.use_snap,
            scene.tool_settings.use_snap_grid_absolute
        ]

    @classmethod
    def reset_settings(cls, scene):
        grid_scale = cls.reset_settings[0]
        for area in bpy.data.screens["Scripting"].areas:
            if area.type == 'VIEW_3D':
                for space in area.spaces:
                    if space.type == 'VIEW_3D':
                        space.overlay.grid_scale = grid_scale if grid_scale is not None else 1
                        break

        scene.tool_settings.snap_elements = cls.reset_settings[1]
        scene.tool_settings.use_snap = cls.reset_settings[2]
        scene.tool_settings.use_snap_grid_absolute = cls.reset_settings[3]


    @classmethod
    def set_snap_parameters(cls, scene, grid: []):
        for area in bpy.data.screens["Scripting"].areas:
            if area.type == 'VIEW_3D':
                for space in area.spaces:
                    if space.type == 'VIEW_3D':
                        space.overlay.grid_scale = grid[0]
                        break

        scene.tool_settings.use_snap = True
        scene.tool_settings.use_snap_grid_absolute = True
        scene.tool_settings.snap_elements = {'INCREMENT'}
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