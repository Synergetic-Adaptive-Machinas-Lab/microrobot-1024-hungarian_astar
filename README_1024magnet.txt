================================================================================
  README: 1024 Magnet Field Simulator — Data Structure and Mapping Reference
================================================================================

File: 1024magnet.html
Based on: direct source code analysis


────────────────────────────────────────────────────────────────────────────────
1. Overview
────────────────────────────────────────────────────────────────────────────────

  - Total magnets        : 1024  (32 rows × 32 columns)
  - Layout               : Hexagonal offset grid
  - State array type     : Int8Array, length 1024
  - Per-magnet state     : +1 (North), -1 (South), 0 (Off)
  - CSV output range     : integer -7 to +7 (polarity only, always full drive)


────────────────────────────────────────────────────────────────────────────────
2. Internal ID Scheme (state array index)
────────────────────────────────────────────────────────────────────────────────

  Each magnet is assigned an integer id from 0 to 1023.
  IDs are assigned in row-major order (left to right, top to bottom).

    id = row * 32 + col

  Inverse:
    row = floor(id / 32)
    col = id % 32

  Examples:
    id = 0    → row = 0,  col = 0   (top-left corner)
    id = 31   → row = 0,  col = 31  (top-right corner)
    id = 32   → row = 1,  col = 0   (start of second row)
    id = 1023 → row = 31, col = 31  (bottom-right corner)

  Screen orientation:
    - row increases downward  (increasing y)
    - col increases rightward (increasing x)
    - id = 0 is the top-left magnet on screen


────────────────────────────────────────────────────────────────────────────────
3. Physical Coordinate Layout (Hexagonal Offset)
────────────────────────────────────────────────────────────────────────────────

  Each magnet's physical position in mm:

    x(row, col) = (col + offset) × 20 mm
    y(row, col) =  row × 17.32 mm

    offset = 0.5 if row is odd, 0.0 if row is even

  Constants:
    Magnet diameter  = 20 mm
    Horizontal pitch = 20 mm
    Vertical pitch   = 20 × √3/2 ≈ 17.32 mm

  Odd rows (1, 3, 5, ...) are shifted 10 mm to the right relative to even rows.
  This creates a close-packed hexagonal offset grid.

  Coordinate examples:
    row=0, col=0  →  x =  0.00 mm,  y =  0.00 mm
    row=0, col=1  →  x = 20.00 mm,  y =  0.00 mm
    row=1, col=0  →  x = 10.00 mm,  y = 17.32 mm   ← odd row: +10 mm offset
    row=1, col=1  →  x = 30.00 mm,  y = 17.32 mm
    row=2, col=0  →  x =  0.00 mm,  y = 34.64 mm   ← even row: no offset

  Grid extent:
    x: −10 mm to ≈ 650 mm
    y: −10 mm to ≈ 541 mm


────────────────────────────────────────────────────────────────────────────────
4. CSV Output Order (SAM LAB Hardware Mapping)
────────────────────────────────────────────────────────────────────────────────

  The CSV payload order differs from the internal id order.
  It follows the SAM LAB hardware protocol: column-major, bottom-to-top.

  Payload index formula:

    payloadIndex = col × 32 + (31 − row)

  Interpretation:
    - Columns advance first (col 0 → 31), then within each column
    - Within a column: row 31 (bottom) fills index 0, row 0 (top) fills index 31

  So:
    payload[0]    = magnet at (row=31, col=0)   ← physical bottom-left
    payload[1]    = magnet at (row=30, col=0)
    ...
    payload[31]   = magnet at (row=0,  col=0)   ← physical top-left
    payload[32]   = magnet at (row=31, col=1)   ← second column, bottom
    ...
    payload[63]   = magnet at (row=0,  col=1)
    ...
    payload[992]  = magnet at (row=31, col=31)  ← physical bottom-right
    ...
    payload[1023] = magnet at (row=0,  col=31)  ← physical top-right

  Summary: "left column to right, each column from bottom to top"


────────────────────────────────────────────────────────────────────────────────
5. Internal ID ↔ Payload Index Mapping
────────────────────────────────────────────────────────────────────────────────

  Internal id (row-major, top-left → bottom-right):
    id = row × 32 + col

  Payload index (column-major, bottom → top):
    payloadIndex = col × 32 + (31 − row)

  These are completely different orderings; conversion is required before
  sending to hardware.

  Corner mappings:
    id = 0    (row=0,  col=0)  → payload[31]    top-left magnet
    id = 31   (row=0,  col=31) → payload[1023]  top-right magnet
    id = 992  (row=31, col=0)  → payload[0]     bottom-left magnet
    id = 1023 (row=31, col=31) → payload[992]   bottom-right magnet


────────────────────────────────────────────────────────────────────────────────
6. CSV File Format
────────────────────────────────────────────────────────────────────────────────

  The exported CSV has 32 rows × 32 columns.

    - Each line: 32 comma-separated integers
    - Total lines: 32
    - Total values: 1024

  What each line represents:
    Line 1  = physical column 0 magnets (bottom row → top row)
    Line 2  = physical column 1 magnets (bottom row → top row)
    ...
    Line 32 = physical column 31 magnets (bottom row → top row)

  One CSV line corresponds to one physical vertical column in the simulator.

  Value range: −7 to +7 (integer)
    Positive (+7) = North pole, full drive
    Negative (−7) = South pole, full drive
    Zero     ( 0) = Off

  Value calculation (PWM fixed at 100%):
    value = state[id] × 7   →   +7, −7, or 0


────────────────────────────────────────────────────────────────────────────────
7. Module / Channel Notation (UI display only)
────────────────────────────────────────────────────────────────────────────────

  When a magnet is selected in the UI, module and channel are shown.
  These are derived from the internal id:

    module  = floor(id / 32) + 1   (1 to 32)
    channel = id % 32              (0 to 31)

  Module corresponds to the row group; channel corresponds to the column.

    module=1,  channel=0  → id=0   (row=0, col=0)
    module=1,  channel=31 → id=31  (row=0, col=31)
    module=32, channel=0  → id=992 (row=31, col=0)


────────────────────────────────────────────────────────────────────────────────
8. Bz Field Model (Measured Radial Lookup Table)
────────────────────────────────────────────────────────────────────────────────

  Bz at the probe plane (z = 25 mm above each electromagnet) is computed from
  a measured radial profile, not an analytic dipole formula.

  Profile values (from dynamics.txt, BZ_RADIAL_PROFILE = mean of BZ_TABLE cols 3&4):

    Radial distance (mm) |  Bz (Gauss)
    ---------------------+------------
          0              |   4.8720
         10              |   3.0408
         20              |   1.1703
         30              |   0.2632

  Lookup and interpolation:
    - Linear interpolation between table points
    - r ≥ 39 mm (EXTERNAL_RANGE): Bz = 0

  Total Bz at a point (x, y):
    Bz_total(x,y) = Σ_i  state[i] × bzTable( sqrt((x−x_i)² + (y−y_i)²) )

  The summation runs over all magnets with non-zero state.


────────────────────────────────────────────────────────────────────────────────
9. Force Model (from dynamics.txt)
────────────────────────────────────────────────────────────────────────────────

  Magnetic force on a ferrofluid disk at position (x, y):

    F = DISK_MOMENT × ∇Bz(x, y)

  where:
    DISK_MOMENT = 1.7 × 10⁻⁵  A·m²  (measured disk magnetic moment)
    ∇Bz computed by finite differences on the Bz total field

  This replaces the earlier dipole-based χ/μ model.


────────────────────────────────────────────────────────────────────────────────
10. Full Data Flow Summary
────────────────────────────────────────────────────────────────────────────────

  [User click / drag on canvas]
        ↓
  state[id] updated  (+1 / −1 / 0,   id = row×32 + col)
        ↓
  buildOrderedPayloads() called
        ↓
  Iterate col = 0..31, within each col row = 31..0
  payloadIndex = col×32 + (31−row)
  value = state[id] × 7
        ↓
  payloadTx[0..1023] complete
        ↓
  Split into 32 groups of 32 → 32 CSV lines → download .csv file

================================================================================
