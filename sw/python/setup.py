from setuptools import setup, find_packages

setup(
    name='Torque Limited Simple Pendulum',
    author= 'Underactuated LAB at DFKI Bremen',
    version='0.1.0',
    #url=
    packages=find_packages(),
    install_requires=[
        #general
        'numpy',
        'pyyaml',
        'pandas',
        'argparse',

        # reinforcement learning
        'stable_baselines'
    ],
    classifiers=[
          'Development Status :: 0.1.0 - Alpha',
          'Environment :: Console',
          'Intended Audience :: Academic Usage',
          'Programming Language :: Python',
          ],
)
