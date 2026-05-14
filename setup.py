from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    name="multiros",
    packages=['multiros'],
    package_dir={'': 'src'},
    # PEP 561: ship the py.typed marker so type checkers (mypy /
    # pyright) treat installed copies of the package as typed.
    package_data={'multiros': ['py.typed']},

    description="MultiROS: ROS-Based Robot Simulation Environment for Concurrent Deep Reinforcement Learning",
    url="https://github.com/ncbdrck/multiros",
    keywords=['ROS', 'reinforcement learning', 'gazebo', 'simulation', 'robotics', 'gym', 'openai'],

    author='Jayasekara Kapukotuwa',
    author_email='j.kapukotuwa@research.ait.ie',

    license="MIT",
)

setup(**setup_args)
