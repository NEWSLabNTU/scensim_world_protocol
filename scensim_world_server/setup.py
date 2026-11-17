from setuptools import setup

NAME = "scensim_world_server"

setup(
    name=NAME,
    version="0.1.0",
    packages=[NAME],
    # the protocol's descriptor set, which protocol.py reads (scripts/gen_world_proto.sh)
    package_data={NAME: ["scensim_world.desc"]},
    data_files=[("share/ament_index/resource_index/packages", [f"resource/{NAME}"]),
                (f"share/{NAME}", ["package.xml"])],
    zip_safe=False,
    install_requires=["setuptools"],
    tests_require=["pytest"],
)
