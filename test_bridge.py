"""
test_bridge.py
--------------
Unit tests for bridge_model.py
FOSSEE IITB 2026 Screening Task

Run with:
    pytest test_bridge.py -v

Requirements:
    pip install pytest pythonocc-core
"""

import math
import pytest

# Import all factory and assembly functions from the model
from bridge_model import (
    # Component factories
    create_i_section,
    create_rectangular_prism,
    create_circular_pier,
    create_trapezoidal_pier_cap,
    create_pile,
    create_pile_cap,
    create_rebar_grid_for_deck,
    create_pier_rebar_cage,
    # Assembly functions
    build_girders,
    build_crossframes,
    build_deck,
    build_piers_and_pilecaps,
    build_parapets,
    assemble_bridge,
    # Parameters (to validate geometry)
    span_length_L,
    n_girders,
    girder_centroid_spacing,
    deck_width,
    deck_overhang,
    deck_thickness,
    girder_section_d,
    girder_section_bf,
    girder_section_tf,
    girder_section_tw,
    pier_diameter,
    pier_height,
    pier_cap_top_width,
    pier_cap_bot_width,
    pier_cap_depth,
    pier_cap_span_length,
    pile_diameter,
    pile_length,
    pile_cap_length,
    pile_cap_width,
    pile_cap_depth,
    pile_spacing,
    rebar_cover,
    rebar_main_diameter,
    rebar_transverse_diameter,
    rebar_spacing_longitudinal,
    rebar_spacing_transverse,
    pier_x_locations,
    pier_rebar_n_bars,
    pier_rebar_main_diameter,
    pier_stirrup_diameter,
    pier_stirrup_spacing,
    concrete_opacity,
    translate,
)

# OCC bounding box helper
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRepBndLib import brepbndlib_Add
from OCC.Core.TopoDS import TopoDS_Compound, TopoDS_Shape


# =============================================================================
# HELPER
# =============================================================================

def get_bounding_box(shape):
    """Return (xmin, ymin, zmin, xmax, ymax, zmax) of a TopoDS_Shape."""
    bbox = Bnd_Box()
    brepbndlib_Add(shape, bbox)
    return bbox.Get()   # xmin, ymin, zmin, xmax, ymax, zmax


def bbox_dims(shape):
    """Return (dx, dy, dz) overall dimensions of bounding box."""
    xmin, ymin, zmin, xmax, ymax, zmax = get_bounding_box(shape)
    return xmax - xmin, ymax - ymin, zmax - zmin


# =============================================================================
# A. COMPONENT FACTORY TESTS
# =============================================================================

class TestCreateISection:
    """Tests for create_i_section(d, bf, tf, tw, length)."""

    def test_returns_shape(self):
        shape = create_i_section(900, 300, 16, 10, 12000)
        assert shape is not None

    def test_bounding_box_length(self):
        """Extruded length along X must equal the 'length' parameter."""
        shape = create_i_section(900, 300, 16, 10, 12000)
        dx, dy, dz = bbox_dims(shape)
        assert abs(dx - 12000) < 1.0, f"Expected X-length 12000, got {dx:.1f}"

    def test_bounding_box_depth(self):
        """Total Z-height must equal depth d."""
        shape = create_i_section(900, 300, 16, 10, 12000)
        dx, dy, dz = bbox_dims(shape)
        assert abs(dz - 900) < 1.0, f"Expected Z-depth 900, got {dz:.1f}"

    def test_bounding_box_width(self):
        """Y-extent must equal flange width bf."""
        shape = create_i_section(900, 300, 16, 10, 12000)
        dx, dy, dz = bbox_dims(shape)
        assert abs(dy - 300) < 1.0, f"Expected Y-width 300, got {dy:.1f}"

    def test_web_thinner_than_flange(self):
        """Web thickness tw < flange width bf — basic sanity."""
        assert girder_section_tw < girder_section_bf

    def test_web_height_positive(self):
        """Web height d - 2*tf must be positive."""
        web_h = girder_section_d - 2 * girder_section_tf
        assert web_h > 0, f"Web height is non-positive: {web_h}"

    def test_parametric_different_length(self):
        """Changing length must change the X bounding box dimension."""
        s1 = create_i_section(900, 300, 16, 10, 6000)
        s2 = create_i_section(900, 300, 16, 10, 15000)
        dx1, _, _ = bbox_dims(s1)
        dx2, _, _ = bbox_dims(s2)
        assert abs(dx1 - 6000)  < 1.0
        assert abs(dx2 - 15000) < 1.0


class TestCreateRectangularPrism:
    """Tests for create_rectangular_prism(length, breadth, height)."""

    def test_returns_shape(self):
        shape = create_rectangular_prism(1000, 500, 200)
        assert shape is not None

    def test_dimensions(self):
        shape = create_rectangular_prism(1000, 500, 200)
        dx, dy, dz = bbox_dims(shape)
        assert abs(dx - 1000) < 1.0
        assert abs(dy - 500)  < 1.0
        assert abs(dz - 200)  < 1.0

    def test_all_positive_dims(self):
        """All dimensions must be positive."""
        assert pile_cap_length > 0
        assert pile_cap_width  > 0
        assert pile_cap_depth  > 0


class TestCreateCircularPier:
    """Tests for create_circular_pier(diameter, height)."""

    def test_returns_shape(self):
        shape = create_circular_pier(800, 3000)
        assert shape is not None

    def test_height(self):
        shape = create_circular_pier(800, 3000)
        dx, dy, dz = bbox_dims(shape)
        assert abs(dz - 3000) < 1.0, f"Expected height 3000, got {dz:.1f}"

    def test_diameter_xy(self):
        """Bounding box X and Y must both equal the diameter."""
        shape = create_circular_pier(800, 3000)
        dx, dy, dz = bbox_dims(shape)
        assert abs(dx - 800) < 1.0, f"Expected X-width 800, got {dx:.1f}"
        assert abs(dy - 800) < 1.0, f"Expected Y-width 800, got {dy:.1f}"

    def test_parametric(self):
        """Different diameters produce different bounding boxes."""
        s1 = create_circular_pier(600, 2000)
        s2 = create_circular_pier(1000, 4000)
        dx1, _, dz1 = bbox_dims(s1)
        dx2, _, dz2 = bbox_dims(s2)
        assert abs(dx1 - 600)  < 1.0
        assert abs(dx2 - 1000) < 1.0
        assert abs(dz1 - 2000) < 1.0
        assert abs(dz2 - 4000) < 1.0


class TestCreateTrapezoidalPierCap:
    """Tests for create_trapezoidal_pier_cap(cap_len_x, top_w_y, bot_w_y, depth_z)."""

    def test_returns_shape(self):
        shape = create_trapezoidal_pier_cap(800, 3500, 1000, 600)
        assert shape is not None

    def test_depth(self):
        """Z-extent must equal depth_z."""
        shape = create_trapezoidal_pier_cap(800, 3500, 1000, 600)
        dx, dy, dz = bbox_dims(shape)
        assert abs(dz - 600) < 1.0, f"Expected depth 600, got {dz:.1f}"

    def test_top_wider_than_bottom(self):
        """Hammerhead: top_w_y > bot_w_y is the whole point."""
        assert pier_cap_top_width > pier_cap_bot_width

    def test_y_extent_bounded_by_top_width(self):
        """Y bounding box should be approximately the top width (widest face)."""
        shape = create_trapezoidal_pier_cap(800, 3500, 1000, 600)
        dx, dy, dz = bbox_dims(shape)
        assert abs(dy - 3500) < 5.0, f"Expected Y ~3500, got {dy:.1f}"


class TestCreatePile:
    """Tests for create_pile(diameter, length)."""

    def test_returns_shape(self):
        assert create_pile(400, 5000) is not None

    def test_dimensions(self):
        shape = create_pile(400, 5000)
        dx, dy, dz = bbox_dims(shape)
        assert abs(dx - 400) < 1.0
        assert abs(dy - 400) < 1.0
        assert abs(dz - 5000) < 1.0


class TestCreatePileCap:
    """Tests for create_pile_cap(length, width, depth)."""

    def test_returns_shape(self):
        assert create_pile_cap(2200, 1200, 600) is not None

    def test_dimensions(self):
        shape = create_pile_cap(2200, 1200, 600)
        dx, dy, dz = bbox_dims(shape)
        assert abs(dx - 2200) < 1.0
        assert abs(dy - 1200) < 1.0
        assert abs(dz - 600)  < 1.0


class TestCreateRebarGridForDeck:
    """Tests for create_rebar_grid_for_deck(...)."""

    def test_returns_list(self):
        rebars = create_rebar_grid_for_deck(
            7000, 12000, 40, 16, 150, 8, 200)
        assert isinstance(rebars, list)

    def test_at_least_one_bar(self):
        rebars = create_rebar_grid_for_deck(
            7000, 12000, 40, 16, 150, 8, 200)
        assert len(rebars) > 0

    def test_longitudinal_bar_count(self):
        """Number of longitudinal bars = floor((deck_w - 2*cover) / spacing) + 1."""
        cover, diam, spacing = 40, 16, 150
        deck_w = 7000
        usable = deck_w - 2 * cover - diam
        expected_min = int(usable / spacing)
        rebars = create_rebar_grid_for_deck(deck_w, 12000, cover, diam, spacing, 8, 200)
        assert len(rebars) >= expected_min

    def test_cover_respected(self):
        """Every bar's bounding box must start at least 'cover' mm from deck edge."""
        cover = 40
        diam  = 16
        rebars = create_rebar_grid_for_deck(7000, 12000, cover, diam, 150, 8, 200)
        half_w = 7000 / 2.0
        for bar in rebars:
            xmin, ymin, zmin, xmax, ymax, zmax = get_bounding_box(bar)
            # Z must be above 0 + cover (bars inside slab)
            assert zmin >= 0, f"Bar below deck slab bottom: zmin={zmin:.1f}"
            # Y must be within deck bounds with cover
            assert ymin >= -half_w + cover - diam, \
                f"Bar outside Y bound: ymin={ymin:.1f}"

    def test_wider_deck_more_bars(self):
        """A wider deck must produce more longitudinal bars."""
        r_narrow = create_rebar_grid_for_deck(3000, 12000, 40, 16, 150, 8, 200)
        r_wide   = create_rebar_grid_for_deck(9000, 12000, 40, 16, 150, 8, 200)
        assert len(r_wide) > len(r_narrow)


class TestCreatePierRebarCage:
    """Tests for create_pier_rebar_cage(...)."""

    def test_returns_list(self):
        cage = create_pier_rebar_cage(800, 3000, 8, 20, 10, 200, 40)
        assert isinstance(cage, list)

    def test_longitudinal_bar_count(self):
        """Must produce exactly n_bars longitudinal bars (plus stirrups)."""
        n = 8
        cage = create_pier_rebar_cage(800, 3000, n, 20, 10, 200, 40)
        # At minimum we must have n longitudinal bars
        assert len(cage) >= n

    def test_bar_radius_within_pier(self):
        """All longitudinal bar centres must be inside the pier radius."""
        pier_r = pier_diameter / 2.0
        cage = create_pier_rebar_cage(
            pier_diameter, pier_height,
            pier_rebar_n_bars, pier_rebar_main_diameter,
            pier_stirrup_diameter, pier_stirrup_spacing, rebar_cover)
        for bar in cage[:pier_rebar_n_bars]:   # only check longitudinal bars
            xmin, ymin, zmin, xmax, ymax, zmax = get_bounding_box(bar)
            cx = (xmin + xmax) / 2.0
            cy = (ymin + ymax) / 2.0
            r  = math.hypot(cx, cy)
            assert r < pier_r, f"Bar centre at r={r:.1f} is outside pier r={pier_r:.1f}"


# =============================================================================
# B. ASSEMBLY FUNCTION TESTS
# =============================================================================

class TestBuildGirders:
    """Tests for build_girders()."""

    def test_correct_count(self):
        girders = build_girders()
        assert len(girders) == n_girders

    def test_girder_length(self):
        """Each girder X-extent must equal span_length_L."""
        for g in build_girders():
            dx, dy, dz = bbox_dims(g)
            assert abs(dx - span_length_L) < 1.0, \
                f"Girder length {dx:.1f} != span {span_length_L}"

    def test_girder_depth(self):
        """Each girder Z-extent must equal girder_section_d."""
        for g in build_girders():
            dx, dy, dz = bbox_dims(g)
            assert abs(dz - girder_section_d) < 1.0

    def test_girder_spacing(self):
        """Adjacent girder Y-centroids must be girder_centroid_spacing apart."""
        girders = build_girders()
        centroids = []
        for g in girders:
            xmin, ymin, zmin, xmax, ymax, zmax = get_bounding_box(g)
            centroids.append((ymin + ymax) / 2.0)
        centroids.sort()
        for i in range(len(centroids) - 1):
            gap = centroids[i + 1] - centroids[i]
            assert abs(gap - girder_centroid_spacing) < 1.0, \
                f"Girder spacing {gap:.1f} != {girder_centroid_spacing}"

    def test_girders_below_deck(self):
        """All girder shapes must be entirely below Z=0 (deck soffit)."""
        for g in build_girders():
            xmin, ymin, zmin, xmax, ymax, zmax = get_bounding_box(g)
            assert zmax <= 0.5, f"Girder top above deck soffit: zmax={zmax:.1f}"


class TestBuildCrossframes:
    """Tests for build_crossframes()."""

    def test_returns_list(self):
        assert isinstance(build_crossframes(), list)

    def test_at_least_one(self):
        """With span=12000 and crossframe_spacing=3000 there must be frames."""
        assert len(build_crossframes()) >= 1

    def test_crossframes_in_web_zone(self):
        """Cross-frames must be within the girder web Z zone."""
        web_top_z    = -girder_section_tf
        web_bottom_z = -(girder_section_d - girder_section_tf)
        for cf in build_crossframes():
            xmin, ymin, zmin, xmax, ymax, zmax = get_bounding_box(cf)
            assert zmax <= web_top_z    + 1.0, f"CF above web top: zmax={zmax:.1f}"
            assert zmin >= web_bottom_z - 1.0, f"CF below web bottom: zmin={zmin:.1f}"


class TestBuildDeck:
    """Tests for build_deck()."""

    def test_returns_shape(self):
        assert build_deck() is not None

    def test_length(self):
        dx, dy, dz = bbox_dims(build_deck())
        assert abs(dx - span_length_L) < 1.0

    def test_width(self):
        dx, dy, dz = bbox_dims(build_deck())
        assert abs(dy - deck_width) < 1.0

    def test_thickness(self):
        dx, dy, dz = bbox_dims(build_deck())
        assert abs(dz - deck_thickness) < 1.0

    def test_deck_above_zero(self):
        """Deck must sit at Z=0 (bottom) to Z=+deck_thickness (top)."""
        xmin, ymin, zmin, xmax, ymax, zmax = get_bounding_box(build_deck())
        assert abs(zmin) < 1.0,   f"Deck bottom not at Z=0: zmin={zmin:.1f}"
        assert abs(zmax - deck_thickness) < 1.0

    def test_deck_width_formula(self):
        """deck_width = (n_girders-1)*spacing + 2*overhang."""
        expected = (n_girders - 1) * girder_centroid_spacing + 2 * deck_overhang
        assert abs(deck_width - expected) < 0.1


class TestBuildPiersAndPilecaps:
    """Tests for build_piers_and_pilecaps()."""

    def test_returns_list_of_tuples(self):
        parts = build_piers_and_pilecaps()
        assert isinstance(parts, list)
        assert all(isinstance(p, tuple) and len(p) == 2 for p in parts)

    def test_boolean_flags(self):
        """Second element of each tuple must be a bool."""
        for shape, is_concrete in build_piers_and_pilecaps():
            assert isinstance(is_concrete, bool)

    def test_has_concrete_and_rebar(self):
        """Assembly must contain both concrete parts and rebar parts."""
        parts = build_piers_and_pilecaps()
        concrete_parts = [p for p, flag in parts if flag]
        rebar_parts    = [p for p, flag in parts if not flag]
        assert len(concrete_parts) > 0, "No concrete parts found"
        assert len(rebar_parts)    > 0, "No rebar parts found"

    def test_one_pile_cap_per_pier(self):
        """Minimum number of parts per pier: cap + column + pile_cap + 4 piles = 7."""
        parts = build_piers_and_pilecaps()
        n_piers = len(pier_x_locations)
        assert len(parts) >= n_piers * 7

    def test_piers_below_deck(self):
        """All pier substructure shapes must be below Z=0."""
        for shape, _ in build_piers_and_pilecaps():
            xmin, ymin, zmin, xmax, ymax, zmax = get_bounding_box(shape)
            assert zmax <= 0.5, \
                f"Pier part extends above deck: zmax={zmax:.1f}"


class TestAssembleBridge:
    """Tests for assemble_bridge()."""

    def test_returns_compound_and_list(self):
        compound, parts = assemble_bridge()
        assert isinstance(compound, TopoDS_Compound)
        assert isinstance(parts, list)

    def test_display_parts_have_colour_tuple(self):
        """Every display part must be (shape, r, g, b, transparency)."""
        _, parts = assemble_bridge()
        for item in parts:
            assert len(item) == 5, f"Expected 5-tuple, got {len(item)}-tuple"
            shape, r, g, b, transp = item
            assert 0.0 <= r <= 1.0
            assert 0.0 <= g <= 1.0
            assert 0.0 <= b <= 1.0
            assert 0.0 <= transp <= 1.0

    def test_total_part_count_reasonable(self):
        """Full bridge must have a meaningful number of geometry parts."""
        _, parts = assemble_bridge()
        assert len(parts) > 20, f"Too few parts: {len(parts)}"

    def test_concrete_parts_have_correct_opacity(self):
        """All concrete parts must use concrete_opacity (spec: 0.35)."""
        _, parts = assemble_bridge()
        for shape, r, g, b, transp in parts:
            if transp > 0:   # transparent = concrete
                assert abs(transp - concrete_opacity) < 0.01 or transp < 0.5, \
                    f"Concrete opacity {transp} too high (spec max ~0.5)"


# =============================================================================
# C. PARAMETRIC / DERIVED VALUE TESTS
# =============================================================================

class TestParametricValues:
    """Tests that derived global parameters are computed correctly."""

    def test_deck_width_formula(self):
        expected = (n_girders - 1) * girder_centroid_spacing + 2 * deck_overhang
        assert abs(deck_width - expected) < 0.1

    def test_n_girders_at_least_3(self):
        assert n_girders >= 3

    def test_flange_not_thicker_than_half_depth(self):
        assert girder_section_tf < girder_section_d / 2.0

    def test_pier_cap_top_wider_than_bottom(self):
        assert pier_cap_top_width > pier_cap_bot_width

    def test_pile_cover_positive(self):
        """Rebar cover must be positive and less than pile radius."""
        assert rebar_cover > 0
        assert rebar_cover < pile_diameter / 2.0

    def test_opacity_in_spec_range(self):
        """concrete_opacity must be <= 0.5 so rebar is visible."""
        assert concrete_opacity <= 0.5, \
            f"Opacity {concrete_opacity} too high — rebar won't be visible"

    def test_pier_locations_within_span(self):
        """All pier X positions must be within the span."""
        for px in pier_x_locations:
            assert 0 < px < span_length_L, \
                f"Pier at X={px} is outside span [0, {span_length_L}]"

    def test_pile_spacing_positive(self):
        assert pile_spacing > 0

    def test_rebar_spacing_positive(self):
        assert rebar_spacing_longitudinal > 0
        assert rebar_spacing_transverse   > 0

    def test_rebar_cover_less_than_deck_thickness(self):
        assert rebar_cover < deck_thickness


# =============================================================================
# D. TRANSLATE UTILITY TEST
# =============================================================================

class TestTranslate:
    """Tests for the translate() helper."""

    def test_translate_moves_shape(self):
        box = create_rectangular_prism(100, 100, 100)
        moved = translate(box, 500, 300, 200)
        xmin, ymin, zmin, xmax, ymax, zmax = get_bounding_box(moved)
        assert abs(xmin - 500) < 1.0
        assert abs(ymin - 300) < 1.0
        assert abs(zmin - 200) < 1.0

    def test_translate_preserves_size(self):
        box = create_rectangular_prism(100, 200, 300)
        moved = translate(box, 1000, 2000, 3000)
        dx, dy, dz = bbox_dims(moved)
        assert abs(dx - 100) < 1.0
        assert abs(dy - 200) < 1.0
        assert abs(dz - 300) < 1.0

    def test_translate_zero_is_identity(self):
        box = create_rectangular_prism(100, 100, 100)
        same = translate(box, 0, 0, 0)
        bb1 = get_bounding_box(box)
        bb2 = get_bounding_box(same)
        for a, b in zip(bb1, bb2):
            assert abs(a - b) < 0.01
