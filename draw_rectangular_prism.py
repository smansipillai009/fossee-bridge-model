import sys
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Display.SimpleGui import init_display
from OCC.Core.gp import gp_Pnt


def create_rectangular_prism(length, breadth, height):
    # Create a rectangular prism using BRepPrimAPI_MakeBox
    box = BRepPrimAPI_MakeBox(length, breadth, height).Shape()
    return box


def display_prism(box):
    # Initialize the display window
    display, start_display, add_menu, add_function_to_menu = init_display()

    # Display the box
    display.DisplayShape(box, update=True)

    # Start the display loop
    start_display()


if __name__ == "__main__":
    # Dimensions of the rectangular prism
    length = 40.0
    breadth = 20.0
    height = 100.0

    # Create the rectangular prism
    box = create_rectangular_prism(length, breadth, height)

    # Display the rectangular prism
    display_prism(box)