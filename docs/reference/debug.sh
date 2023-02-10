#!/bin/bash

# Clean
make clean

# Compile raising warnings as errors
make html SPHINXOPTS="-W"
