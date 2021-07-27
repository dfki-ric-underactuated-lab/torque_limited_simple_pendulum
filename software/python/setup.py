from setuptools import setup, find_packages

setup(
    name='TorqueLimitedSimplePendulum',
    author='Underactuated LAB at DFKI Bremen',
    version='1.0.0',
    url="https://github.com/dfki-ric-underactuated-lab",
    packages=find_packages(),
    install_requires=[
        # general
        'numpy',
        'pyyaml',
        'pandas',
        'argparse',

        # reinforcement learning
        'stable_baselines'
    ],
    classifiers=[
          'Development Status :: 5 - Stable',
          'Environment :: Console',
          'Intended Audience :: Academic Usage',
          'Programming Language :: Python',
          ],
)
