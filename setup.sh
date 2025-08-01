#!/bin/bash

set -e

ENV_DIR=".venv"

# Create venv
echo "Creating virtualenv..."
python3 -m venv $ENV_DIR
source $ENV_DIR/bin/activate

# Install packages
echo "Installing packages from requirements.txt..."
pip install --upgrade pip
pip install -r requirements.txt


echo "To activate the environment, run: source $ENV_DIR/bin/activate"