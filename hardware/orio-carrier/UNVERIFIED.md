# Do not fabricate

Generated with unverified pin tables:

- `tools/morpho_map.py` (Nucleo Morpho headers)
- `tools/part_pinouts.py` -> `AHCT244`
- `tools/part_pinouts.py` -> `AP2112K`
- `tools/part_pinouts.py` -> `B0505S`
- `tools/part_pinouts.py` -> `ISO7721`
- `tools/part_pinouts.py` -> `LM5145`
- `tools/part_pinouts.py` -> `LM74700`
- `tools/part_pinouts.py` -> `TPS54360B`

The netlist is correct by signal name. Only the physical pin numbers are pending.
Fix the tables, re-run `python tools/generate.py`, and this file disappears.

