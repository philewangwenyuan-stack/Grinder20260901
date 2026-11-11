"""Verify packaged scene references and complete binary STL assets without ROS."""
from pathlib import Path
import struct
import unittest
import xml.etree.ElementTree as ET


class BundledSceneTest(unittest.TestCase):
    def test_world_and_every_mesh_resolve_inside_packaged_scene(self):
        root = Path(__file__).resolve().parents[1] / "scenes/huadu_factory_gazebo"
        world = ET.parse(str(root / "worlds/huadu_factory.world"))
        self.assertEqual("model://huadu_factory", world.find(".//include/uri").text)
        model = root / "models/huadu_factory"
        document = ET.parse(str(model / "model.sdf"))
        references = {node.text for node in document.findall(".//mesh/uri")}
        self.assertGreater(len(references), 1)
        for uri in references:
            self.assertTrue(uri.startswith("model://huadu_factory/"))
            mesh = model / uri[len("model://huadu_factory/"):]
            self.assertTrue(mesh.is_file(), uri)
            with mesh.open("rb") as stream:
                header = stream.read(84)
            triangles = struct.unpack("<I", header[80:84])[0]
            self.assertEqual(84 + 50 * triangles, mesh.stat().st_size, uri)


if __name__ == "__main__":
    unittest.main()
