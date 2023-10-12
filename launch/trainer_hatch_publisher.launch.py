from launch import LaunchDescription
from clr_trainer_hatch_offsets.launch_common import load_transforms

def generate_launch_description():

    declared_arguments = []

    site_config_path = "trainer_hatch_transforms/"

    bench_nodes = load_transforms("clr_trainer_hatch_offsets", "config/" + site_config_path + "bench_transforms.yaml")
    hatch_4040_nodes = load_transforms("clr_trainer_hatch_offsets", "config/" + site_config_path + "hatch_4040_transforms.yaml")
    hatch_4060_nodes = load_transforms("clr_trainer_hatch_offsets", "config/" + site_config_path + "hatch_4060_transforms.yaml")
    intermediate_nodes = load_transforms("clr_trainer_hatch_offsets", "config/" + site_config_path + "intermediate_transforms.yaml")


    nodes_to_launch = [
        *bench_nodes,
        *hatch_4040_nodes,
        *hatch_4060_nodes,
        *intermediate_nodes
    ]

    return LaunchDescription(declared_arguments + nodes_to_launch)

