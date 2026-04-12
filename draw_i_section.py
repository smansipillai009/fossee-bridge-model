from OCC.Core.gp import gp_Vec, gp_Trsf
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Display.SimpleGui import init_display


def create_i_section(length, width, depth, flange_thickness, web_thickness):
    """
    Create an I-section CAD model with the specified dimensions.

    Parameters:
    - length: Length of the I-section
    - width: Width of the I-section (horizontal dimension)
    - height: Total height of the I-section (vertical dimension)
    - flange_thickness: Thickness of the flanges
    - web_thickness: Thickness of the web

    Returns:
    - i_section_solid: The I-section CAD model as a TopoDS_Solid
    """
    # Dimensions for the I-section
    web_height = depth - 2 * flange_thickness

    # Create the bottom flange
    bottom_flange = BRepPrimAPI_MakeBox(length, width, flange_thickness).Shape()

    # Create the top flange
    top_flange = BRepPrimAPI_MakeBox(length, width, flange_thickness).Shape()
    trsf = gp_Trsf()
    trsf.SetTranslation(gp_Vec(0, 0, depth - flange_thickness))  # Move to the top
    top_flange_transform = BRepBuilderAPI_Transform(top_flange, trsf, True).Shape()

    # Create the web
    web = BRepPrimAPI_MakeBox(length, web_thickness, web_height).Shape()
    trsf = gp_Trsf()
    trsf.SetTranslation(gp_Vec(0, (width - web_thickness) / 2, flange_thickness))  # Centered between flanges
    web_transform = BRepBuilderAPI_Transform(web, trsf, True).Shape()

    # Combine the flanges and web to form the I-section
    i_section_solid = BRepAlgoAPI_Fuse(bottom_flange, top_flange_transform).Shape()
    i_section_solid = BRepAlgoAPI_Fuse(i_section_solid, web_transform).Shape()

    return i_section_solid


if __name__ == "__main__":
    length = 1000.0
    width = 100.0  # Width of the I-section
    height = 200.0  # Height of the I-section
    flange_thickness = 10.0  # Thickness of the flanges
    web_thickness = 5.0  # Thickness of the web

    i_section = create_i_section(length, width, height, flange_thickness, web_thickness)

    # Visualization
    display, start_display, add_menu, add_function_to_menu = init_display()

    # Show the I-section model
    display.DisplayShape(i_section, update=True)
    display.FitAll()
    start_display()