import unittest
from mockups_launch_common.yaml_loader import load_tf_transforms


class TestUtility(unittest.TestCase):
    def test_load_yaml(self):
        package_name = "mockups_launch_common"
        data_file = "config/example_transforms.yaml"

        # Sanity check to make sure the correct transforms are loaded
        transforms = load_tf_transforms(package_name, data_file)
        self.assertEqual(len(transforms), 8)

        tf1 = transforms[0]
        self.assertEqual(tf1.child_frame_id, "new_transform_1")
        self.assertEqual(tf1.header.frame_id, "parent_transform_1")
        self.assertAlmostEqual(tf1.transform.translation.x, 0.05)
        self.assertAlmostEqual(tf1.transform.translation.y, -0.09)
        self.assertAlmostEqual(tf1.transform.translation.z, 0.0)
        self.assertAlmostEqual(tf1.transform.rotation.x, 0.7071067)
        self.assertAlmostEqual(tf1.transform.rotation.y, 0.0)
        self.assertAlmostEqual(tf1.transform.rotation.z, -0.7071067)
        self.assertAlmostEqual(tf1.transform.rotation.w, 0.0)


if __name__ == "__main__":
    unittest.main()
