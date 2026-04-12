"""
bridge_model.py
---------------
Parametric 3D CAD Model of a Steel Girder Bridge using pythonOCC
Developed for FOSSEE IITB 2026 Screening Task

Coordinate System:
  X-axis -> Longitudinal direction (span)
  Y-axis -> Transverse direction (deck width)
  Z-axis -> Vertical direction
  Origin -> Centre of span at bottom face of deck slab (Z = 0)  [as per spec]

  Z levels (top-down):
    Deck top             : Z = +deck_thickness
    Deck bottom / girder top : Z = 0
    Girder bottom        : Z = -girder_section_d
    Pier cap top         : Z = -girder_section_d
    Pier cap bottom      : Z = -(girder_section_d + pier_cap_depth)
    Pier column top      : Z = -(girder_section_d + pier_cap_depth)
    Pier column bottom   : Z = -(girder_section_d + pier_cap_depth + pier_height)
    Pile cap top         : same as pier column bottom
    Pile cap bottom      : Z = pier_col_bot - pile_cap_depth
    Pile tips            : Z = pile_cap_bot - pile_length

Units: mm (all dimensions in millimetres)

Author  : [Mansi Pillai]
Date    : 11-04-2026
"""

import math
import argparse

from OCC.Core.gp import (gp_Vec, gp_Trsf, gp_Pnt, gp_Ax2, gp_Dir, gp_Ax1)
from OCC.Core.BRepPrimAPI import (BRepPrimAPI_MakeBox,
                                   BRepPrimAPI_MakeCylinder,
                                   BRepPrimAPI_MakePrism)
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.BRepBuilderAPI import (BRepBuilderAPI_Transform,
                                      BRepBuilderAPI_MakePolygon,
                                      BRepBuilderAPI_MakeWire,
                                      BRepBuilderAPI_MakeEdge,
                                      BRepBuilderAPI_MakeFace)
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCC.Core.TopoDS import TopoDS_Compound
from OCC.Core.BRep import BRep_Builder
from OCC.Core.BRepTools import breptools_Write
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Display.SimpleGui import init_display
from OCC.Core.Quantity import Quantity_Color, Quantity_TOC_RGB


# PARAMETERS - change these to adjust the entire model
# All values in millimetres unless noted.

units                      = "mm"

# --- Span & Layout ---
span_length_L              = 12000   # Total span length (mm)
n_girders                  = 3       # Number of main longitudinal girders (>= 3)
girder_centroid_spacing    = 3000    # Centre-to-centre spacing between girders (mm)
deck_overhang              = 500     # Deck overhang beyond outer girders (mm)
# Derived recomputed after CLI parsing
deck_width                 = (n_girders - 1) * girder_centroid_spacing + 2 * deck_overhang  # 7000

# --- Pier locations (derived from span; recomputed after CLI) ---
# Two piers placed at 1/4 and 3/4 of span length
pier_x_locations           = [span_length_L * 0.25, span_length_L * 0.75]

# --- Cross-frames ---
crossframe_spacing         = 3000    # Spacing along X between cross-frames (mm)
crossframe_depth           = 600     # Depth of cross-frame plate (mm)
crossframe_thickness       = 20      # Thickness of cross-frame plate (mm)

# --- Main Girder (I-section) ---
girder_section_d           = 900     # Total depth of I-section (mm)
girder_section_bf          = 300     # Flange width (mm)
girder_section_tf          = 16      # Flange thickness (mm)
girder_section_tw          = 10      # Web thickness (mm)
girder_length              = span_length_L
girder_material            = "Steel IS 2062 E250"

# --- Deck Slab ---
deck_thickness             = 200     # Deck slab thickness (mm)
deck_cover                 = 40      # Concrete cover for rebar (mm)
deck_slab_segment_length   = None    # Optional: segment deck into prisms of this length (None=one piece)
n_lanes                    = 2       # Optional: number of traffic lanes (for documentation)
lane_width                 = 3500    # Optional: lane width mm (for documentation)
concrete_opacity           = 0.5    # 0 = opaque, 1 = invisible

# --- Pier ---
pier_diameter              = 800     # Pier column diameter (mm)
pier_height                = 3000    # Pier column height (mm)
pier_cap_top_width         = 3500    # Transverse (Y) width at top of hammerhead cap (mm)
pier_cap_bot_width         = 1000    # Transverse (Y) width at bottom of cap (mm)
pier_cap_depth             = 600     # Vertical depth of pier cap (mm)
pier_cap_span_length       = 800     # Along-span (X) bearing length of pier cap (mm)

# --- Pier column reinforcement ---
pier_rebar_n_bars          = 8       # Number of longitudinal bars around perimeter
pier_rebar_main_diameter   = 20      # Longitudinal bar diameter (mm)
pier_stirrup_diameter      = 10      # Stirrup diameter (mm)
pier_stirrup_spacing       = 200     # Stirrup spacing along column height (mm)

# --- Pile & Pile Cap ---
pile_diameter              = 400     # Pile diameter (mm)
pile_length                = 5000    # Pile length (mm)
pile_spacing               = 600     # Pile centre-to-centre spacing (mm)
n_piles_per_cap            = 4       # Number of piles per cap (arrange in sqrt grid, e.g. 4=2x2)
pile_cap_length            = 2200    # Pile cap length along X (mm)
pile_cap_width             = 1200    # Pile cap width along Y (mm)
pile_cap_depth             = 600     # Pile cap depth along Z (mm)

# --- Reinforcement (deck slab) ---
rebar_main_diameter        = 16      # Longitudinal rebar diameter (mm)
rebar_transverse_diameter  = 8       # Transverse/stirrup rebar diameter (mm)
rebar_spacing_longitudinal = 150     # Spacing of longitudinal bars (mm)
rebar_spacing_transverse   = 200     # Spacing of transverse bars (mm)
rebar_cover                = 40      # Concrete cover (mm)
rebar_visible              = True    # Toggle rebar display

# --- Parapets / Railings (optional) ---
show_parapets              = True
parapet_height             = 1000    # Height of parapet wall (mm)
parapet_thickness          = 200     # Thickness of parapet wall (mm)

# --- Visualization ---
show_axes                  = True
background_gradient_top    = [220, 225, 235]
background_gradient_bot    = [170, 185, 205]

# --- Export ---
save_step                  = False
step_filename              = "bridge_assembly.step"
save_brep                  = False
brep_filename              = "bridge_assembly.brep"


# CLI ARGUMENT PARSING - allows overriding parameters at runtime

def parse_args():
    """Parse CLI arguments and update global parameters accordingly."""
    global span_length_L, n_girders, girder_centroid_spacing
    global crossframe_spacing, concrete_opacity, show_parapets
    global rebar_visible, save_step, save_brep
    global deck_width, girder_length, pier_x_locations

    parser = argparse.ArgumentParser(
        description="Parametric Steel Girder Bridge — FOSSEE IITB 2026 Screening Task")
    parser.add_argument("--span",               type=float, help="Span length in mm (default 12000)")
    parser.add_argument("--n-girders",          type=int,   help="Number of girders (default 3, min 3)")
    parser.add_argument("--girder-spacing",     type=float, help="Girder centre-to-centre spacing mm")
    parser.add_argument("--crossframe-spacing", type=float, help="Cross-frame spacing along span mm")
    parser.add_argument("--opacity",            type=float, help="Concrete opacity 0 (opaque) to 1 (invisible)")
    parser.add_argument("--no-rebar",           action="store_true", help="Hide rebar")
    parser.add_argument("--no-parapets",        action="store_true", help="Hide parapets")
    parser.add_argument("--save-step",          action="store_true", help="Export STEP file")
    parser.add_argument("--save-brep",          action="store_true", help="Export BREP file")

    args = parser.parse_args()

    if args.span is not None:
        span_length_L = args.span
    if args.n_girders is not None:
        n_girders = max(3, args.n_girders)
    if args.girder_spacing is not None:
        girder_centroid_spacing = args.girder_spacing
    if args.crossframe_spacing is not None:
        crossframe_spacing = args.crossframe_spacing
    if args.opacity is not None:
        concrete_opacity = args.opacity
    if args.no_rebar:
        rebar_visible = False
    if args.no_parapets:
        show_parapets = False
    if args.save_step:
        save_step = True
    if args.save_brep:
        save_brep = True

    deck_width       = (n_girders - 1) * girder_centroid_spacing + 2 * deck_overhang
    girder_length    = span_length_L
    pier_x_locations = [span_length_L * 0.25, span_length_L * 0.75]


# HELPER UTILITIES

def translate(shape, dx, dy, dz):
    """Translate a TopoDS shape by (dx, dy, dz) mm."""
    trsf = gp_Trsf()
    trsf.SetTranslation(gp_Vec(dx, dy, dz))
    return BRepBuilderAPI_Transform(shape, trsf, True).Shape()


# COMPONENT FACTORIES  (named exactly as required by the problem statement)

def create_i_section(d, bf, tf, tw, length):
    """
    I-section solid extruded along X-axis.

    Parameters
    ----------
    d      : float  Total depth (mm)
    bf     : float  Flange width (mm)
    tf     : float  Flange thickness (mm)
    tw     : float  Web thickness (mm)
    length : float  Extrusion length along X (mm)

    Geometry (local coords, caller translates to final position)
    --------
    Bottom flange : Z = 0      to tf,    Y = -bf/2 to +bf/2
    Web           : Z = tf     to d-tf,  Y = -tw/2 to +tw/2
    Top flange    : Z = d-tf   to d,     Y = -bf/2 to +bf/2
    """
    web_h = d - 2 * tf

    bot = BRepPrimAPI_MakeBox(length, bf, tf).Shape()
    bot = translate(bot, 0, -bf / 2.0, 0)

    top = BRepPrimAPI_MakeBox(length, bf, tf).Shape()
    top = translate(top, 0, -bf / 2.0, d - tf)

    web = BRepPrimAPI_MakeBox(length, tw, web_h).Shape()
    web = translate(web, 0, -tw / 2.0, tf)

    sec = BRepAlgoAPI_Fuse(bot, top).Shape()
    sec = BRepAlgoAPI_Fuse(sec, web).Shape()
    return sec


def create_rectangular_prism(length, breadth, height):
    """
    Rectangular prism with one corner at origin.

    Parameters
    ----------
    length  : float  Dimension along X (mm)
    breadth : float  Dimension along Y (mm)
    height  : float  Dimension along Z (mm)
    """
    return BRepPrimAPI_MakeBox(length, breadth, height).Shape()


def create_circular_pier(diameter, height):
    """
    Vertical circular cylinder (pier column).
    Base disk centred at origin (X=0, Y=0, Z=0), axis along +Z.

    Parameters
    ----------
    diameter : float  Column diameter (mm)
    height   : float  Column height (mm)
    """
    return BRepPrimAPI_MakeCylinder(diameter / 2.0, height).Shape()


def create_trapezoidal_pier_cap(cap_len_x, top_w_y, bot_w_y, depth_z):
    """
    Hammerhead / trapezoidal pier cap via ThruSections loft.

    Parameters
    ----------
    cap_len_x : float  Along-span (X) bearing length of cap (mm)  e.g. 800
    top_w_y   : float  Transverse (Y) width at top               (mm)  e.g. 3500
    bot_w_y   : float  Transverse (Y) width at bottom            (mm)  e.g. 1000
    depth_z   : float  Vertical depth                            (mm)  e.g. 600

    The cap is centred at X=0, Y=0.
    Bottom face at Z=0, top face at Z=depth_z.
    Caller translates to final position.
    """
    def rect_wire(half_x, half_y, z):
        poly = BRepBuilderAPI_MakePolygon()
        poly.Add(gp_Pnt(-half_x, -half_y, z))
        poly.Add(gp_Pnt( half_x, -half_y, z))
        poly.Add(gp_Pnt( half_x,  half_y, z))
        poly.Add(gp_Pnt(-half_x,  half_y, z))
        poly.Close()
        return poly.Wire()

    loft = BRepOffsetAPI_ThruSections(True, True)   # solid=True, ruled=True
    loft.AddWire(rect_wire(cap_len_x / 2.0, bot_w_y / 2.0, 0.0))
    loft.AddWire(rect_wire(cap_len_x / 2.0, top_w_y / 2.0, depth_z))
    loft.Build()
    return loft.Shape()


def create_pile(diameter, length):
    """
    Circular pile solid. Base centred at origin, extends along +Z.
    Caller translates downward.

    Parameters
    ----------
    diameter : float  Pile diameter (mm)
    length   : float  Pile length (mm)
    """
    return BRepPrimAPI_MakeCylinder(diameter / 2.0, length).Shape()


def create_pile_cap(length, width, depth):
    """
    Rectangular pile cap solid. Corner at origin.

    Parameters
    ----------
    length : float  Along X (mm)
    width  : float  Along Y (mm)
    depth  : float  Along Z (mm)
    """
    return create_rectangular_prism(length, width, depth)


def create_rebar_grid_for_deck(deck_w, span_len, cover, diam, spacing,
                                trans_diam=None, trans_spacing=None):
    """
    Rebar grid (longitudinal + transverse) inside a deck slab volume.

    Parameters
    ----------
    deck_w        : float  Total deck width (mm)
    span_len      : float  Span length (mm)
    cover         : float  Concrete cover from slab edge (mm)
    diam          : float  Longitudinal rebar diameter (mm)
    spacing       : float  Spacing of longitudinal bars along Y (mm)
    trans_diam    : float  Transverse rebar diameter (mm)  [optional]
    trans_spacing : float  Spacing of transverse bars along X (mm) [optional]

    Returns
    -------
    list of TopoDS_Shape  — one shape per bar
    """
    if trans_diam is None:
        trans_diam = diam
    if trans_spacing is None:
        trans_spacing = spacing

    rebars = []
    z_long  = cover + diam / 2.0
    z_trans = z_long + diam   # slightly above longitudinal layer

    # Longitudinal bars — along X, spaced in Y
    y = -deck_w / 2.0 + cover + diam / 2.0
    while y <= deck_w / 2.0 - cover - diam / 2.0:
        axis = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0))
        bar = BRepPrimAPI_MakeCylinder(axis, diam / 2.0, span_len).Shape()
        bar = translate(bar, 0, y, z_long)
        rebars.append(bar)
        y += spacing

    # Transverse bars  along Y, spaced in X
    x = cover + trans_diam / 2.0
    while x <= span_len - cover - trans_diam / 2.0:
        axis = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0))
        bar = BRepPrimAPI_MakeCylinder(axis, trans_diam / 2.0,
                                        deck_w - 2 * cover).Shape()
        bar = translate(bar, x, -deck_w / 2.0 + cover, z_trans)
        rebars.append(bar)
        x += trans_spacing

    return rebars


def create_pier_rebar_cage(pier_diam, height, n_bars,
                            long_diam, stirrup_diam, stirrup_spacing, cover):
    """
    Rebar cage for a circular pier column.

    Parameters
    ----------
    pier_diam       : float  Pier outer diameter (mm)
    height          : float  Pier height (mm)
    n_bars          : int    Number of longitudinal bars around perimeter
    long_diam       : float  Longitudinal bar diameter (mm)
    stirrup_diam    : float  Stirrup bar diameter (mm)
    stirrup_spacing : float  Stirrup spacing along height (mm)
    cover           : float  Concrete cover (mm)

    Returns
    -------
    list of TopoDS_Shape
    """
    rebars = []
    bar_r = pier_diam / 2.0 - cover - stirrup_diam - long_diam / 2.0

    # Longitudinal bars equally spaced around circumference
    for i in range(n_bars):
        angle = 2 * math.pi * i / n_bars
        cx = bar_r * math.cos(angle)
        cy = bar_r * math.sin(angle)
        bar = BRepPrimAPI_MakeCylinder(long_diam / 2.0, height).Shape()
        bar = translate(bar, cx, cy, 0)
        rebars.append(bar)

    # Stirrups dodecagonal wire rings extruded to thin discs
    ring_r  = bar_r + long_diam / 2.0 + stirrup_diam / 2.0
    n_sides = 12
    z = stirrup_spacing
    while z < height - stirrup_spacing:
        pts = []
        for k in range(n_sides):
            a = 2 * math.pi * k / n_sides
            pts.append(gp_Pnt(ring_r * math.cos(a),
                              ring_r * math.sin(a),
                              z))
        pts.append(pts[0])  # close ring

        wire_b = BRepBuilderAPI_MakeWire()
        for k in range(n_sides):
            wire_b.Add(BRepBuilderAPI_MakeEdge(pts[k], pts[k + 1]).Edge())

        if wire_b.IsDone():
            face = BRepBuilderAPI_MakeFace(wire_b.Wire(), True)
            if face.IsDone():
                stirrup = BRepPrimAPI_MakePrism(
                    face.Shape(), gp_Vec(0, 0, stirrup_diam)).Shape()
                rebars.append(stirrup)
            else:
                print(f"  Warning: stirrup face failed at z={z:.0f}")
        z += stirrup_spacing

    return rebars


# ASSEMBLY FUNCTIONS  (named exactly as required by the problem statement)

def build_girders():
    """
    Create and position n_girders I-section girders.

    Girders hang below the deck:
      Z = -girder_section_d  (bottom of bottom flange)
      Z = 0                  (top of top flange = deck soffit)

    Returns list of TopoDS_Shape.
    """
    shapes = []
    y_start = -((n_girders - 1) * girder_centroid_spacing) / 2.0
    for i in range(n_girders):
        yc = y_start + i * girder_centroid_spacing
        g = create_i_section(girder_section_d, girder_section_bf,
                             girder_section_tf, girder_section_tw,
                             girder_length)
        g = translate(g, 0, yc - girder_section_bf / 2.0, -girder_section_d)
        shapes.append(g)
    return shapes


def build_crossframes():
    """
    Transverse cross-frame plates spanning between outer girders,
    positioned at regular X intervals in the web zone.

    Returns list of TopoDS_Shape.
    """
    shapes = []
    web_h    = girder_section_d - 2 * girder_section_tf
    cf_depth = min(crossframe_depth, web_h - 10)  
    cf_span  = (n_girders - 1) * girder_centroid_spacing

    # Z position: centred in web zone
    web_mid_z = -girder_section_d + girder_section_tf + web_h / 2.0
    cf_bot_z  = web_mid_z - cf_depth / 2.0

    x = crossframe_spacing
    while x < girder_length - crossframe_spacing / 2.0:
        cf = create_rectangular_prism(crossframe_thickness, cf_span, cf_depth)
        cf = translate(cf,
                       x - crossframe_thickness / 2.0,
                       -cf_span / 2.0,
                       cf_bot_z)
        shapes.append(cf)
        x += crossframe_spacing
    return shapes


def build_deck():
    """
    Concrete deck slab: Z = 0 (bottom) to Z = +deck_thickness (top).
    Centred in Y.

    Returns TopoDS_Shape.
    """
    d = create_rectangular_prism(span_length_L, deck_width, deck_thickness)
    return translate(d, 0, -deck_width / 2.0, 0)


def build_piers_and_pilecaps():
    """
    Build the complete substructure for all piers:
      pier cap (trapezoidal loft)   - pier column (cylinder)  
      pile cap (rectangular)       - piles (2×2 cylinders)
    Rebar cages included in pier column and pile cap when rebar_visible=True.

    Returns list of (TopoDS_Shape, is_concrete: bool)
      is_concrete=True - concrete colour + opacity
      is_concrete=False - steel/rebar colour + opaque
    """
    parts = []
    for px in pier_x_locations:
        parts.extend(_build_single_pier(px))
    return parts


def _build_single_pier(px):
    """Building one full pier assembly at longitudinal position X = px."""
    parts = []

    # --- Z levels (top-down) ---
    pier_cap_top_z = -girder_section_d
    pier_cap_bot_z = pier_cap_top_z - pier_cap_depth
    pier_top_z     = pier_cap_bot_z
    pier_bot_z     = pier_top_z - pier_height
    pile_cap_top_z = pier_bot_z
    pile_cap_bot_z = pile_cap_top_z - pile_cap_depth
    pile_bot_z     = pile_cap_bot_z - pile_length

    # --- Pier cap (trapezoidal hammerhead) ---
    cap = create_trapezoidal_pier_cap(pier_cap_span_length,
                                      pier_cap_top_width,
                                      pier_cap_bot_width,
                                      pier_cap_depth)
    # Loft is centred at (0,0,0)
    cap = translate(cap, px, 0, pier_cap_bot_z)
    parts.append((cap, True))

    # Pier cap rebar - longitudinal bars inside cap
    if rebar_visible:
        pc_cover = 50
        bar_z = pier_cap_bot_z + pc_cover + rebar_main_diameter / 2.0
        for y_bar in [-pier_cap_top_width / 4.0, 0, pier_cap_top_width / 4.0]:
            axis = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0))
            bar = BRepPrimAPI_MakeCylinder(
                axis, rebar_main_diameter / 2.0, pier_cap_span_length).Shape()
            bar = translate(bar, px - pier_cap_span_length / 2.0, y_bar, bar_z)
            parts.append((bar, False))

    # --- Pier column ---
    pier = create_circular_pier(pier_diameter, pier_height)
    pier = translate(pier, px, 0, pier_bot_z)
    parts.append((pier, True))

    # Pier column rebar cage
    if rebar_visible:
        cage = create_pier_rebar_cage(
            pier_diameter, pier_height,
            pier_rebar_n_bars, pier_rebar_main_diameter,
            pier_stirrup_diameter, pier_stirrup_spacing,
            rebar_cover
        )
        for bar in cage:
            bar = translate(bar, px, 0, pier_bot_z)
            parts.append((bar, False))

    # --- Pile cap ---
    pc = create_pile_cap(pile_cap_length, pile_cap_width, pile_cap_depth)
    pc = translate(pc,
                   px - pile_cap_length / 2.0,
                   -pile_cap_width / 2.0,
                   pile_cap_bot_z)
    parts.append((pc, True))

    # Pile cap rebar - bottom two-way mat
    if rebar_visible:
        pc_cover = 50
        z_bot = pile_cap_bot_z + pc_cover + rebar_main_diameter / 2.0
        # Bars along X
        y_b = -pile_cap_width / 2.0 + pc_cover + rebar_main_diameter / 2.0
        while y_b <= pile_cap_width / 2.0 - pc_cover:
            axis = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0))
            bar = BRepPrimAPI_MakeCylinder(
                axis, rebar_main_diameter / 2.0, pile_cap_length).Shape()
            bar = translate(bar, px - pile_cap_length / 2.0, y_b, z_bot)
            parts.append((bar, False))
            y_b += rebar_spacing_longitudinal
        # Bars along Y
        x_b = px - pile_cap_length / 2.0 + pc_cover + rebar_main_diameter / 2.0
        z_bot2 = z_bot + rebar_main_diameter
        while x_b <= px + pile_cap_length / 2.0 - pc_cover:
            axis = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0))
            bar = BRepPrimAPI_MakeCylinder(
                axis, rebar_main_diameter / 2.0, pile_cap_width).Shape()
            bar = translate(bar, x_b, -pile_cap_width / 2.0, z_bot2)
            parts.append((bar, False))
            x_b += rebar_spacing_longitudinal

    # --- Piles (parametric n_piles_per_cap grid centred at (px, 0)) ---
    # n_piles_per_cap must be a perfect square (e.g. 4=2x2, 9=3x3)
    n_side = max(2, int(round(n_piles_per_cap ** 0.5)))
    offsets = [pile_spacing * (i - (n_side - 1) / 2.0) for i in range(n_side)]
    for ox in offsets:
        for oy in offsets:
            pile = create_pile(pile_diameter, pile_length)
            pile = translate(pile, px + ox, oy, pile_bot_z)
            parts.append((pile, True))

    return parts


def build_parapets():
    """
    Rectangular parapet walls along both longitudinal edges of the deck.
    Sit on top of deck slab (Z = deck_thickness to deck_thickness + parapet_height).

    Returns list of TopoDS_Shape (empty if show_parapets=False).
    """
    if not show_parapets:
        return []
    shapes = []
    # Left parapet at -deck_width/2, right at +deck_width/2 - thickness
    for y_pos in [-deck_width / 2.0,
                   deck_width / 2.0 - parapet_thickness]:
        p = create_rectangular_prism(span_length_L, parapet_thickness, parapet_height)
        p = translate(p, 0, y_pos, deck_thickness)
        shapes.append(p)
    return shapes


def assemble_bridge():
    """
    Assemble all bridge components into a single TopoDS_Compound and
    collect display metadata.

    Returns
    -------
    compound     : TopoDS_Compound - full bridge geometry
    display_parts: list of (TopoDS_Shape, r, g, b, transparency)
    """
    builder  = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)

    display_parts = []   # (shape, r, g, b, transparency)

    def add(shape, r, g, b, transp=0.0):
        builder.Add(compound, shape)
        display_parts.append((shape, r, g, b, transp))

    # Steel colour constants
    R_STEEL, G_STEEL, B_STEEL = 0.40, 0.42, 0.55
    # Concrete colour
    R_CONC,  G_CONC,  B_CONC  = 0.78, 0.76, 0.72
    # Rebar colour
    R_REBAR, G_REBAR, B_REBAR = 0.70, 0.15, 0.10
    # Parapet colour (slightly lighter concrete)
    R_PAR,   G_PAR,   B_PAR   = 0.82, 0.80, 0.76

    print("  [1/6] Building girders...")
    for g in build_girders():
        add(g, R_STEEL, G_STEEL, B_STEEL, 0.0)

    print("  [2/6] Building cross-frames...")
    for cf in build_crossframes():
        add(cf, R_STEEL, G_STEEL, B_STEEL, 0.0)

    print("  [3/6] Building deck slab...")
    add(build_deck(), R_CONC, G_CONC, B_CONC, concrete_opacity)

    if rebar_visible:
        print("  [4/6] Building deck rebar...")
        for r in create_rebar_grid_for_deck(
                deck_width, span_length_L,
                rebar_cover,
                rebar_main_diameter, rebar_spacing_longitudinal,
                rebar_transverse_diameter, rebar_spacing_transverse):
            add(r, R_REBAR, G_REBAR, B_REBAR, 0.0)
    else:
        print("  [4/6] Deck rebar skipped (--no-rebar)")

    print("  [5/6] Building pier substructure...")
    for (shape, is_concrete) in build_piers_and_pilecaps():
        if is_concrete:
            add(shape, R_CONC, G_CONC, B_CONC, concrete_opacity)
        else:
            add(shape, R_REBAR, G_REBAR, B_REBAR, 0.0)

    print("  [6/6] Building parapets...")
    for p in build_parapets():
        add(p, R_PAR, G_PAR, B_PAR, 0.15)

    return compound, display_parts


# EXPORT HELPERS  

def export_step(compound, filename):
    """Export full bridge compound to a STEP file."""
    writer = STEPControl_Writer()
    writer.Transfer(compound, STEPControl_AsIs)
    status = writer.Write(filename)
    if status == IFSelect_RetDone:
        print(f"  STEP exported → {filename}")
    else:
        print(f"  STEP export FAILED (status={status})")


def export_brep(compound, filename):
    """Export full bridge compound to a BREP file."""
    breptools_Write(compound, filename)
    print(f"  BREP exported → {filename}")


# DISPLAY

def display_bridge(display, display_parts):
    """Render each part with its assigned colour and transparency."""
    for (shape, r, g, b, transp) in display_parts:
        color = Quantity_Color(r, g, b, Quantity_TOC_RGB)
        display.DisplayShape(shape, color=color, transparency=transp, update=False)


# MAIN

def main():
    parse_args()   # applying CLI overrides and recompute derived globals

    print("=" * 60)
    print("  Parametric Steel Girder Bridge — FOSSEE IITB 2026")
    print("=" * 60)
    print(f"  Span length      : {span_length_L} mm")
    print(f"  Deck width       : {deck_width} mm")
    print(f"  No. of girders   : {n_girders}")
    print(f"  Pier locations X : {pier_x_locations}")
    print(f"  Rebar visible    : {rebar_visible}")
    print(f"  Concrete opacity : {concrete_opacity}")
    print(f"  Parapets         : {show_parapets}")
    print()

    print("Assembling bridge components...")
    compound, display_parts = assemble_bridge()
    print(f"  Total display parts: {len(display_parts)}")

    if save_step:
        export_step(compound, step_filename)
    if save_brep:
        export_brep(compound, brep_filename)

    # --- Visualize ---
    print("\nInitializing OCC display...")
    display, start_display, add_menu, add_function_to_menu = init_display()
    display.set_bg_gradient_color(background_gradient_top, background_gradient_bot)

    display_bridge(display, display_parts)

    display.FitAll()
    display.View_Iso()  

    if show_axes:
        display.display_triedron()

    display.Repaint()

    print("\nBridge displayed.")
    print("  Left-drag  = rotate   |  Scroll = zoom   |  Middle-drag = pan")
    print(f"\nActive parameters:")
    print(f"  span={span_length_L}mm  girders={n_girders}  "
          f"opacity={concrete_opacity}  rebar={rebar_visible}")
    start_display()


if __name__ == "__main__":
    main()
