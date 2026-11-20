"""The surface classes every world server shares (scensim roadmap 086 C2): the table's path.

The table (`surface_classes.yaml`, beside this module) gives each ground class its friction,
wet factors, tyre curve, hydroplaning and soil, and the source each was calibrated against.
A world server reads it with its own loader and uses what its simulator can; the file is also
installed to `share/scensim_world_server/config/` for a world server that is not Python.
"""
from pathlib import Path

SURFACE_CLASSES = Path(__file__).resolve().parent / "surface_classes.yaml"
