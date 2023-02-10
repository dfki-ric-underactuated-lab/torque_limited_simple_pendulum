from setuptools import setup, find_packages

setup(
    name='TorqueLimitedSimplePendulum',
    author='Underactuated Lab DFKI Robotics Innovation Center Bremen',
    version='1.0.0',
    url="https://github.com/dfki-ric-underactuated-lab",
    packages=find_packages(),
    install_requires=[
        # general
        'numpy',
        'pyyaml',
        'pandas',
        'argparse',
        'scipy',
        'sympy',
        'mini-cheetah-motor-driver-socketcan',
        #'eigenpy',
        #'inputs',
        'evdev',
        'scikit-learn',

        ## optimal control
        #'drake',

        ## reinforcement learning
        #'tensorflow>=2.6.0',
        #'pickle5',
        #'stable_baselines3',

        ## documentation
        #'sphinx',
        #'sphinx-rtd-theme',
        #'numpydoc',

        ## testing
        #'pytest',
        #'lark',

    ],
    extras_require={
        "all": ['drake', 'tensorflow>=2.6.0', 'stable_baselines3',
                'sphinx', 'sphinx-rtd-theme', 'numpydoc', 'pytest', 'lark'],
        "drake": ['drake'],
        "tensorflow": ['tensorflow>=2.6.0'],
        "stable_baselines": ['stable_baselines3'],
        "doc": ['sphinx', 'sphinx-rtd-theme', 'numpydoc'],
        "test": ['pytest', 'lark']
    },
    classifiers=[
          'Development Status :: 5 - Stable',
          'Environment :: Console',
          'Intended Audience :: Academic Usage',
          'Programming Language :: Python',
          ],
)
