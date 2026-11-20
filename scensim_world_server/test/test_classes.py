"""The shared surface class table is where classes.py says, and is a table (roadmap 086 C2)."""
import os
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scensim_world_server import classes  # noqa: E402


def test_the_table_is_beside_the_module_and_names_its_reference():
    data = yaml.safe_load(classes.SURFACE_CLASSES.read_text())
    table = data["classes"]
    assert data["reference"] in table
    for name, c in table.items():
        assert isinstance(c["mu"], (int, float)) and c["mu"] > 0, name
        assert isinstance(c.get("wet", {}), dict), name


def test_setup_installs_it_as_package_data_and_to_share():
    text = open(os.path.join(os.path.dirname(__file__), "..", "setup.py")).read()
    assert '"surface_classes.yaml"' in text
    assert 'f"share/{NAME}/config"' in text
