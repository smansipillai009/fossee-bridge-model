# Parametric 3D CAD Model of a Steel Girder Bridge

**FOSSEE IITB 2026 Screening Task — pythonOCC**

A fully parametric, modular 3D CAD model of a short-span steel girder bridge built with the [pythonOCC](https://github.com/tpaviot/pythonocc-core) (pythonocc-core) library. Every dimension is driven by named parameters at the top of `bridge_model.py` — change one value and the entire bridge updates automatically.

---

<img width="1919" height="1097" alt="Screenshot 2026-04-24 160822" src="https://github.com/user-attachments/assets/465d433c-a36c-4250-804b-2d15828c6e75" />


## Features

- **3 or more I-section steel girders** with correct `d`, `bf`, `tf`, `tw` geometry
- **Concrete deck slab** — semi-transparent so internal reinforcement is visible
- **Transverse cross-frames** between every adjacent girder pair at regular intervals
- **Trapezoidal hammerhead pier caps** (proper loft geometry via `ThruSections`)
- **Circular pier columns** with full rebar cage (longitudinal bars + dodecagonal stirrups)
- **Rectangular pile caps** with two-way bottom reinforcement mat
- **2×2 pile groups** beneath each pier cap
- **Parapets/railings** along both deck edges
- **CLI parameter overrides** — change span, girder count, opacity, etc. at runtime
- **STEP and BREP export** of the full assembly compound
- **Isometric camera preset** with axis triedron

---

## Bridge Components

| Component | Type | Factory Function |
| Main girders | I-section solid | `create_i_section()` |
| Deck slab | Rectangular prism | `create_rectangular_prism()` |
| Pier columns | Circular cylinder | `create_circular_pier()` |
| Pier cap | Trapezoidal loft | `create_trapezoidal_pier_cap()` |
| Piles | Circular cylinder | `create_pile()` |
| Pile cap | Rectangular prism | `create_pile_cap()` |
| Deck rebar grid | Cylinders | `create_rebar_grid_for_deck()` |
| Pier rebar cage | Cylinders + rings | `create_pier_rebar_cage()` |

---

## Coordinate System

```
X-axis  →  Longitudinal direction (span)
Y-axis  →  Transverse direction (deck width)
Z-axis  →  Vertical direction
Origin  →  Start of span, bottom face of deck slab (Z = 0)
```

| Level | Z value |
| Deck top | `+deck_thickness` |
| Deck bottom / girder top | `0` |
| Girder bottom | `-girder_section_d` |
| Pier cap (top → bottom) | `-girder_section_d` to `-(d + pier_cap_depth)` |
| Pier column | below pier cap |
| Pile cap | below pier column |
| Pile tips | lowest |

---

## Default Parameters

All units are **millimetres (mm)**.

```python
span_length_L            = 12000   # Total span length
n_girders                = 3       # Number of main girders (min 3)
girder_centroid_spacing  = 3000    # Centre-to-centre girder spacing
deck_overhang            = 500     # Overhang beyond outer girders
deck_thickness           = 200     # Deck slab thickness
girder_section_d         = 900     # I-section depth
girder_section_bf        = 300     # Flange width
girder_section_tf        = 16      # Flange thickness
girder_section_tw        = 10      # Web thickness
pier_diameter            = 800     # Pier column diameter
pier_height              = 3000    # Pier column height
pier_cap_top_width       = 3500    # Hammerhead cap top width (transverse)
pier_cap_bot_width       = 1000    # Hammerhead cap bottom width
pier_cap_depth           = 600     # Pier cap depth
pile_diameter            = 400     # Pile diameter
pile_length              = 5000    # Pile length
pile_cap_length          = 2200    # Pile cap length
pile_cap_width           = 1200    # Pile cap width
concrete_opacity         = 0.35    # 0 = opaque, 1 = invisible
rebar_visible            = True    # Show/hide rebar
```

---

## Installation

### 1. Install pythonocc-core

The easiest method is via conda:

```bash
conda install -c conda-forge pythonocc-core
```

Or via pip (requires a compatible environment):

```bash
pip install pythonocc-core
```

### 2. Clone this repository

```bash
git clone https://github.com/smansipillai009/fossee-bridge-model.git
cd fossee-bridge-model
```

### 3. Install test dependencies

```bash
pip install pytest
```

---

## Usage

### Run with default parameters

```bash
python bridge_model.py
```

### CLI overrides

```bash
# Change span length
python bridge_model.py --span 15000

# Change number of girders
python bridge_model.py --n-girders 4

# Adjust concrete transparency
python bridge_model.py --opacity 0.5

# Hide rebar (faster render)
python bridge_model.py --no-rebar

# Hide parapets
python bridge_model.py --no-parapets

# Export STEP file
python bridge_model.py --save-step

# Export BREP file
python bridge_model.py --save-brep

# Combine options
python bridge_model.py --span 18000 --n-girders 5 --opacity 0.4 --save-step
```

### Viewer controls

| Action | Control |
|---|---|
| Rotate | Left-drag |
| Zoom | Scroll wheel |
| Pan | Middle-drag |
| Reset view | `V` key |

---

## Running Tests

```bash
pytest test_bridge.py -v
```

### Test coverage

| Category | Tests |
|---|---|
| `create_i_section` | Shape, bounding box (X/Y/Z), parametric |
| `create_rectangular_prism` | Dimensions |
| `create_circular_pier` | Height, diameter, parametric |
| `create_trapezoidal_pier_cap` | Depth, Y extent, top > bottom |
| `create_pile` / `create_pile_cap` | Dimensions |
| `create_rebar_grid_for_deck` | Bar count, cover, wider deck = more bars |
| `create_pier_rebar_cage` | Bar count, bars within pier radius |
| `build_girders` | Count, length, depth, spacing, below Z=0 |
| `build_crossframes` | List, count, within web zone |
| `build_deck` | Length, width, thickness, Z position |
| `build_piers_and_pilecaps` | Tuple format, concrete + rebar present |
| `assemble_bridge` | Compound type, colour tuples, opacity |
| Parameters | Derived values, engineering constraints |
| `translate` | Movement, size preservation |

---

## File Structure

```
fossee-bridge-model/
├── bridge_model.py       # Main parametric model + visualization
├── test_bridge.py        # pytest unit tests
├── README.md             # This file
├── report.pdf            # Documentation report (submitted separately)
```

---

## Code Structure

```
bridge_model.py
│
├── PARAMETERS             ← All dimensions as named globals
├── parse_args()           ← CLI argument parser
├── translate()            ← Helper: move shape in 3D
│
├── Component Factories
│   ├── create_i_section()
│   ├── create_rectangular_prism()
│   ├── create_circular_pier()
│   ├── create_trapezoidal_pier_cap()
│   ├── create_pile()
│   ├── create_pile_cap()
│   ├── create_rebar_grid_for_deck()
│   └── create_pier_rebar_cage()
│
├── Assembly Functions
│   ├── build_girders()
│   ├── build_crossframes()
│   ├── build_deck()
│   ├── build_piers_and_pilecaps()
│   │   └── _build_single_pier()
│   ├── build_parapets()
│   └── assemble_bridge()         ← Returns TopoDS_Compound
│
├── Export
│   ├── export_step()
│   └── export_brep()
│
└── main()                ← Entry point
```

---

## Engineering Notes

- **I-section geometry** follows standard ISMB conventions: `d` = total depth, `bf` = flange width, `tf` = flange thickness, `tw` = web thickness
- **Rebar cover** of 40mm is maintained on all concrete faces per IS 456
- **Pier cap** uses a ruled loft (`ThruSections`) between two rectangular wires — bottom narrower (`pier_cap_bot_width`) and top wider (`pier_cap_top_width`) — creating a realistic hammerhead shape
- **Pier locations** are derived parametrically at 25% and 75% of the span, not hardcoded
- **Cross-frames** span between the inner flange faces of each adjacent girder pair, placed at mid-web height

---

## Dependencies

| Package | Version | Purpose |
|---|---|---|
| `pythonocc-core` | ≥ 7.7 | 3D CAD geometry kernel |
| `pytest` | ≥ 7.0 | Unit testing |
| Python | ≥ 3.8 | Language runtime |

---

## License

MIT License — see [LICENSE](LICENSE) for details.

---

## Author

Mansi Pillai
FOSSEE IITB 2026 Screening Task  
*Osdag — Open Steel Design and Graphics*
