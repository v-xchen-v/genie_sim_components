#!/bin/bash

ENV_NAME=g1_relax_env
WHEEL=../../../../third_party/relaxed_ik/relaxed_ik-0.1.0-py3-none-any.whl
SRC_DIR=$(realpath ../../..)

# Setup conda env
eval "$(conda shell.bash hook)"
conda create -y -n $ENV_NAME python=3.10 || true
conda activate $ENV_NAME

# Install deps
pip install --upgrade pip
pip install scikit-learn
[ -f "$WHEEL" ] && pip install "$WHEEL" || echo "Wheel not found!"

# Set PYTHONPATH
export PYTHONPATH="$SRC_DIR:$PYTHONPATH"
echo "export PYTHONPATH=\"$SRC_DIR:\$PYTHONPATH\"" >> ~/.bashrc

echo "[✅] Setup complete. Environment: $ENV_NAME"