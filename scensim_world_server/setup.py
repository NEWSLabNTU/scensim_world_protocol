from setuptools import setup

NAME = "scensim_world_server"

setup(
    name=NAME,
    version="0.1.0",
    packages=[NAME],
    # the protocol's descriptor set, which protocol.py reads (scripts/gen_world_proto.sh)
    # the surface classes every world server shares (classes.py; roadmap 086 C2)
    package_data={NAME: ["scensim_world.desc", "surface_classes.yaml"]},
    data_files=[("share/ament_index/resource_index/packages", [f"resource/{NAME}"]),
                (f"share/{NAME}", ["package.xml"]),
                (f"share/{NAME}/config", [f"{NAME}/surface_classes.yaml"])],
    zip_safe=False,
    install_requires=["setuptools"],
    tests_require=["pytest"],
)
