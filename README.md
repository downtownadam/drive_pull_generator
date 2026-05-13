# Drive Pulley Generator

Parametric V-groove drive pulley STL generator for 3D printing.

Generates a two-piece pulley that can be printed on its side without supports. Each half prints flat on the split face, then assembles together with alignment pegs.

## Cross-Section Profile

```
        ___________           ___________
       |           |         |           |
       |   flange  |         |   flange  |
       |___________|  /---\  |___________|
                   | / V-  \ |
                   |/ groove\|
        ___________|  valley |___________
       |           |         |           |
       |    shaft (large)    shaft (thd) |
       |___________|_________|___________|

       |<-- large side -->|<- thd side->|
       |<-------- pulley width -------->|
```

## Installation

```bash
pip install -r requirements.txt
```

**Note:** CadQuery requires Python 3.9+ and may need conda for easiest installation:

```bash
conda install -c cadquery cadquery
```

## Usage

Generate with default parameters:

```bash
python pulley_generator.py
```

Custom parameters:

```bash
python pulley_generator.py \
    --pulley-width 25 \
    --shaft-dia-large 12 \
    --shaft-dia-threaded 8 \
    --threaded-side-width 8 \
    --valley-diameter 40 \
    --flange-diameter 60 \
    --groove-angle 38 \
    --set-screw-dia 3 \
    --weight-cutouts \
    --cutout-count 6
```

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `--pulley-width` | 25 mm | Total width of the assembled pulley |
| `--shaft-dia-large` | 12 mm | Bore diameter on the large/keyed side |
| `--shaft-dia-threaded` | 8 mm | Bore diameter on the threaded/small side |
| `--threaded-side-width` | 8 mm | Width of shaft section on threaded side |
| `--valley-diameter` | 40 mm | Diameter at the bottom of the V-groove |
| `--flange-diameter` | 60 mm | Outer diameter at the pulley flanges |
| `--groove-angle` | 38 deg | Included angle of the V-groove |
| `--set-screw-dia` | 3 mm | Set screw hole diameter (0 to disable) |
| `--weight-cutouts` | off | Add triangular weight reduction cutouts |
| `--cutout-count` | 6 | Number of weight reduction cutouts |

## Output

Files are written to `output/` by default:

- `pulley_large_side.stl` - Half with the larger bore
- `pulley_threaded_side.stl` - Half with the smaller bore
- `pulley_assembled.step` - Combined preview (for CAD viewers)

## Printing Tips

- Print each half flat on the split face (the large flat side)
- No supports needed
- 3-4 perimeters recommended for strength
- 30%+ infill for pulleys under load
- PETG or ABS recommended for durability and heat resistance
