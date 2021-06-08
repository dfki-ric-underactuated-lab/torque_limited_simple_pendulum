<div align="center">

#  Simple Pendulum
</div>


# Requirements - Python Code  

## Installing Python 3.7
In order to run the python code within the repository you will need `Python >=3.7, <4` along with the package installer `pip3`. 
  
**Step 1)** First, you might want to see if you already have a suitable version installed. This command shows which Python 3 version is currently set as default:

```
python3 --version
```

If the default Python 3 version does not match the requirements you can list all python versions that are installed on your system under /usr by typing

```
ls /usr/bin/python*
```

**Step 2)** If any suitable version is listed (otherwise directly jump to Step 5), we need to check whether there are any Python alternatives configured. Therefore, execute the command below in the terminal.

```
sudo update-alternatives --list python
```
  
**Step 3)** If there are no matching python alternatives configured, you can add the desired Python version to our alternatives. For example, if version /usr/bin/python3.7 exists on the system, you can add it to your alternatives via

```
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.7 1
```

**Step 4)** After adding python 3.7 to you alternatives you can display it and verfify wether it is set as default with this command

```
sudo update-alternatives --config python
```

Enter the selection number, on the prompt that appears on the terminal, to set python 3.7 as your default version. You have successfully set Python 3 to your default python version and can now jump directly to Step 8.

**Step 5)** In case you haven't encountered a suitable python version on your system you can just install it with apt. Start by updating the packages list and installing the prerequisites:

```
sudo apt update
sudo apt install software-properties-common
```

**Step 6)** Next, add the deadsnakes PPA to your sources list:

```
sudo add-apt-repository ppa:deadsnakes/ppa
```

When prompted press `Enter` to continue:

```
Press [ENTER] to continue or Ctrl-c to cancel adding it.
```

**Step 7)** Once the repository is enabled, install Python 3.7 with

```
sudo apt install python3.7
```

**Step 8)** At this point, Python 3.7 is installed on your Ubuntu system and ready to be used. You can verify it by typing:

```
python3.7 --version
```

## Installing pip3
Once you have  `Python >=3.7, <4` installed update the package list using the following command:

```
sudo apt update
```

and install pip3 via

```
sudo apt install python3-pip
```

If you like, you can verify the installation by checking the pip version:

```
pip3 --version
```

## Creating a Virtual Environment 
The easiest and recommended way to configure your own custom Python environment is via `Virtualenv`. 

**Step 1)** Use pip3 to simply install virtualenv:  

```
pip3 install virtualenv
```

**Step 2)** You'll need the full path to your python and to your virtualenv installation, so run the following to view both paths:

```
which virtualenv
outputs e.g. /home/username/opt/python-3.7.2/bin/virtualenv

which python3
outputs e.g.  /home/username/opt/python-3.7.2/bin/python
```

**Step 3)** Navigate to the directory, where you want to create the new virtual environment. In our case this will be at `~/torque_limited_simple_pendulum/sw/python`. Create the virtual environment, while specifing the desired python version. 

```
virtualenv -p /home/username/opt/python-3.7.2/bin/python3 venv
```

This command creates a virtualenv named 'venv' and uses the -p flag to specify the full path to the Python3 version you just installed. You may see the following error when installing:

```
setuptools pip failed with error code 1` error
```

If so, run the following:

```
pip3 install --upgrade setuptools
```

Try again and you should be able to install without an error.

**Step 4)** Activate the new virtual environment with the command

```
source venv/bin/activate
```

The name of the current virtual environment `(venv)` appears to the left of the prompt, indicating that you are now working inside a virtual environment.

When finished working in the virtual environment, you can deactivate it by running the following:

```
deactivate
```

In case that you don't need the virtual environment anymore, you can deactivate it and remove it together with all previously installed packages:

```
sudo rm -rf venv
```

## Installing Python Packages
Now that you’re in your virtual environment you can install all required packages from the existing requirements.txt file via:

```
python3 -m pip install -r requirements.txt
```

(Note: You can generate your own requirements.txt file with this command: `pip freeze > requirements.txt`)






 
