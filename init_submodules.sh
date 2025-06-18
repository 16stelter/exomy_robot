#!/bin/bash

set -e

echo "Initializing submodules..."
git submodule update --init --recursive

echo "Setting submodules to their configured branches..."
git submodule foreach '
  branch=$(git config -f $toplevel/.gitmodules submodule.$name.branch)
  if [ -z "$branch" ]; then
    echo "No branch configured for $name. Skipping."
  else
    echo "Checking out branch '$branch' in $name..."
    git fetch origin $branch
    git checkout $branch
    git pull origin $branch
  fi
'

