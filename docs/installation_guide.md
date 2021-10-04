<div align="center">

#  Installation Guide
</div>

In order to execute the python code within the repository you will need to have `Python (>=3.6, <4)` along with the package installer `pip3` on your system installed.

- **python (>=3.6, <4)**
- **pip3**

If you aren't running a suitable python version currently on your system, we recommend you to install the required python version inside of an virtual environment for python (**pyenv**) and to install all python packages necessary to use this repo afterwards inside a newly created virtual environment (**virtualenv**). The installation procedure for Ubuntu (18.04.5 and 20.04.2.0 LTS) are described in the next section. You can find instructions for MacOS and other Linux distributions, as well as information about common build problems here:

- **pyenv:** https://github.com/pyenv/pyenv
- **virtualenv:** https://github.com/pyenv/pyenv-virtualenv

## Installing this Python Package
If you want to install this package with your system python version, you can do that by going to the directory [software/python](software/python) and typing:
```
pip install .
```
Note: This has to be repeated if you make changes to the code (besides the scripts).

## Instructions for Ubuntu (18.04.5 and 20.04.2.0 LTS)
The instructions provide assistance in the setup procedure, but with regards to the software [LICENSE](LICENSE) they are provided without warranty of any kind. In no event shall the authors or copyright holders be liable for any claim, damages or other liability, arising from, out of or in connection with the software or the use or other dealings in the software.

1) Clone this repo from GitHub, in case you haven't done it yet:
```
git clone git@git.hb.dfki.de:underactuated-robotics/torque_limited_simple_pendulum.git
```

2) Check your Python version with:

```
python3 --version
```

If you are already using suitable Python 3.6 version jump directly to step `Creating a Virtual Environment` otherwise continue here and first install a virtual environment for python.

<br>

## A) Pyenv: Virtual environment for Python

The following instructions are our recommendations for a sane build environment.

**Step 1)** Make sure to have installed python's binary dependencies and build tools as per

```
sudo apt-get update; sudo apt-get install make build-essential libssl-dev zlib1g-dev \
libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm \
libncursesw5-dev xz-utils tk-dev libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev
```

**Step 2)** Once prerequisites have been installed correctly, install Pyenv with

```
curl https://pyenv.run | bash
```

**Step 3)** Configure your shell's environment for Pyenv

<ins>Note:</ins> The below instructions are designed for common shell setups. If you have an uncommon setup and they don't work for you, use the linked guidance to figure out what you need to do in your specific case: https://github.com/pyenv/pyenv#advanced-configuration

Before editing your `.bashrc` and `.profile` files it is a good idea to <ins>make a copy</ins> of both files in case something goes wrong. Add pyenv to your `.bashrc` file from the terminal:

```
echo 'eval "$(pyenv init -)"' >> ~/.bashrc
```

Add these lines at the beginning of your `.profile` file (not from the terminal):
```
export PYENV_ROOT="$HOME/.pyenv"
export PATH="$PYENV_ROOT/bin:$PATH"
```
and this line at the very end of your `.profile` file (not from the terminal):

```
eval "$(pyenv init --path)"
```

**Step 4)** Source `.profile` and `.bashrc`, then restart your shell so the path changes take effect:

```
source ~/.profile
source ~/.bashrc
```

```
exec $SHELL
```
**Step 5)** Run `pyenv init -` in your shell, then copy and also execute the output to enable shims

```
pyenv init -
```

Restart your login session for the changes to take effect. If you're in a GUI session, you need to fully log out and log back in. You can now begin using pyenv.


<ins>Optional:</ins> Consider uprading to the latest version of Pyenv via

```
pyenv update
```

<br>

## B) Installing Python into Pyenv
You can diplay a list of available Python versions with:

```
pyenv install -l | grep -ow [0-9].[0-9].[0-9]
```

Install your desired Python version using Pyenv (We suggest 3.6.9 for ubuntu 18.04 and 3.8.10 for ubuntu 20.04):

```
pyenv install 3.x.x
```

Double check your work:

```
pyenv versions
```

To use Python 3.x only for this specific project change directory to the cloned git repo and type:

```
pyenv local 3.x.x
```
<br>

## C) Creating a Virtual Environment with Pyenv
In order to clutter your system as little as possible all further packages will be installed inside a virtual environment, which can be easily removed at any time. The recommended way to configure your own custom Python environment is via `Virtualenv`.

**Step 1)** Clone virtualenv from https://github.com/pyenv/pyenv-virtualenv into the pyenv-plugin directory:
```
git clone https://github.com/pyenv/pyenv-virtualenv.git $(pyenv root)/plugins/pyenv-virtualenv
```

**Step 2)** Add pyenv virtualenv-init to your shell to enable auto-activation of virtualenvs:
```
echo 'eval "$(pyenv virtualenv-init -)"' >> ~/.bashrc
```

**Step 3)** Restart your shell to enable pyenv-virtualenv
```
exec "$SHELL"
```

**Step 4)** To create a new virtual environment, e.g. named simple-pendulum with Python 3.6.9 run

```
pyenv virtualenv 3.6.9 simple-pendulum
```

**Step 5)** Activate the new virtual environment with the command

```
pyenv activate simple-pendulum
```

The name of the current virtual environment `(venv)` appears to the left of the prompt, indicating that you are now working inside a virtual environment. When finished working in the virtual environment, you can deactivate it by running the following:

```
pyenv deactivate
```

In case that you don't need the virtual environment anymore, you can deactivate it and remove it together with all previously installed packages:

```
pyenv uninstall simple-pendulum

```

<br>

## D) Installing pip3
Update the package list inside your recently created virtual environment:

```
sudo apt update
```

and install pip3 via

```
sudo apt install python3-pip
```

If you like, you can update pip and verify your the installation by:

```
pip install --upgrade pip
```
```
pip3 --version
```
<br>

## E) Install Requirements for this Repository
Navigate inside your cloned git repo to `/torque_limited_simple_pendulum/software/python` and make sure your virtual environment is active `pyenv activate simple-pendulum`. Now you can install version specific packages for all required packages from the `requirements.txt` file via:

```
python3 -m pip install -r requirements.txt
```

Note 1: You can generate your own requirements.txt file with this command: `pip freeze > requirements.txt`
Note 2: Yoiu can skip this step and directly install this package with the setup.py file as described in the next line. This will install the requirement packages as well. The setup.py will not install specific versions of the requirements, that have been tested by us, but instead, it will install the latest version.

This package then can be installed from the [software/python](software/python) directory by typing:
```
pip install .
```

This was the final installation step. Your system is now prepared to run all code snippets from this repo. Have fun exploring all kind of different simple pendulum controllers!

<br>

## OPTIONAL: Installing Drake for Python
Drake is not installable via pip at present. It is available as a binary package and the latest image can be pulled from Docker Hub simply with: `docker pull robotlocomotion/drake:latest`. 
  

**Check here for more detailed instructions:** https://drake.mit.edu/python_bindings.html 

--------------------------------------------------------------------------------------------------------------

:warning: **Attention**: By following the instructions you will install Drake permanently on your system. This may cause incompatiblity issues with preexisting Python packages. Drake for instance is incompatible with the Python environment supplied by Anaconda. Please uninstall Anaconda or remove the Anaconda bin directory from the PATH before building or using the Drake Python bindings. Before attempting the installation, please review the supported configurations to know what versions of Python are supported for your platform: https://drake.mit.edu/developers.html#supported-configurations
  
--------------------------------------------------------------------------------------------------------------

**Step 1)** Download and extract an available binary package. As an example, here is how to download and extract one of the latest releases to /opt (where `<platform>` could be bionic, focal, or mac):

```
curl -o drake.tar.gz https://drake-packages.csail.mit.edu/drake/nightly/drake-latest-<platform>.tar.gz
sudo rm -rf /opt/drake
sudo tar -xvzf drake.tar.gz -C /opt
```

**Step 2)** Ensure that you have the system dependencies:

```
sudo /opt/drake/share/drake/setup/install_prereqs
```

**Step 3)** Configure your `PYTHONPATH` properly: 

```
# Ubuntu 18.04 (Bionic):
export PYTHONPATH=/opt/drake/lib/python3.6/site-packages:${PYTHONPATH}

# Ubuntu 20.04 (Focal):
export PYTHONPATH=/opt/drake/lib/python3.8/site-packages:${PYTHONPATH}
```

**Step 4)** To check whether the installation was successful you can import `pydrake`:

```
python3 -c 'import pydrake; print(pydrake.__file__)'
```

If this command returns the directory of your pydrake installation, you are now able to import pydrake modules to your python code with e.g. `from pydrake.solvers.mathematicalprogram import Solve`. A comprehensive list of all available pydrake modules is provided by the `Python API documentation`: https://drake.mit.edu/pydrake/index.html
  
<br>    





 
