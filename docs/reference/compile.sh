#!/bin/bash

# Remove any generated documentation
find . -name 'simple_pendulum*rst'
find . -name 'simple_pendulum*rst' -delete
# Clean
make clean

# Generate site
make html
find . -name 'simple_pendulum*rst'
find . -name 'simple_pendulum*rst' -delete
# Generate LaTeX code
make latex
find . -name 'simple_pendulum*rst'
find . -name 'simple_pendulum*rst' -delete

# Build
cd build
cd latex
make LATEXMKOPTS="-lualatex"
