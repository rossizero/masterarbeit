bl_info = {
    "name": "IFC Modeling Helper",
    "blender": (2, 80, 0),
    "category": "Object",
}

import bpy
import blenderbim.tool as tool
import ifcopenshell.util.element


class IfcModelingHelper:
    last_selection = []
    grid = None
    GRID_IDENTIFICATOR = "grid"

    @classmethod
    def look_for_grid(cls, obj):
        # set global grid to the one that's defined by the active ifcwall
        grid = None
        id_ = obj.BIMObjectProperties.ifc_definition_id
        if id_ is not None and id_ != 0:
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
                        void_element = ifc_rel_opening_element.VoidsElements[0]  # [1:1]
                        building_element = void_element.RelatingBuildingElement
                        obj = building_element
                if obj.is_a("IfcOpeningElement"):
                    void_element = obj.VoidsElements[0]  # [1:1]
                    building_element = void_element.RelatingBuildingElement
                    obj = building_element
                if obj.is_a("IfcWall"):
                    obj_type = ifcopenshell.util.element.get_type(obj)
                    grid = cls.get_grid_from_type(obj_type)
        return grid

    @classmethod
    def get_grid_from_type(cls, ifc_type):
        type_psets = ifcopenshell.util.element.get_psets(ifc_type)
        if type_psets is not None:
            for name in type_psets.keys():
                pset = type_psets[name]
                if cls.GRID_IDENTIFICATOR in pset.keys():
                    g = [round(n, 9) for n in pset[cls.GRID_IDENTIFICATOR]]
                    return g
        return None

    @classmethod
    def apply_grid(cls, obj, grid):
        if grid:
            for i, val in enumerate(grid):
                obj.location[i] = round((obj.location[i] / val)) * val
            obj.dimensions = [round((obj.dimensions[i] / val)) * val for i, val in enumerate(grid)]

    @classmethod
    def on_depsgraph_update(cls, scene, _):
        if bpy.context.selected_objects != cls.last_selection:
            cls.last_selection = bpy.context.selected_objects

            if len(bpy.context.selected_objects) != 0:
                obj = bpy.context.selected_objects[0]
                cls.grid = cls.look_for_grid(obj)
                cls.apply_grid(bpy.context.selected_objects[0], cls.grid)

        depsgraph = bpy.context.evaluated_depsgraph_get()

        for update in depsgraph.updates:
            obj = bpy.context.active_object
            # cls.grid = cls.look_for_grid(obj)

            if update.is_updated_transform or update.is_updated_geometry:
                cls.apply_grid(obj, cls.grid)
            if update.is_updated_shading:
                pass


def register():
    bpy.app.handlers.depsgraph_update_post.clear()
    bpy.app.handlers.depsgraph_update_post.append(IfcModelingHelper.on_depsgraph_update)


def unregister():
    bpy.app.handlers.depsgraph_update_post.remove(IfcModelingHelper.on_depsgraph_update)


# This allows to run the script directly from Blender's Text editor
if __name__ == "__main__":
    register()
