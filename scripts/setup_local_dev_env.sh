#!/bin/sh

# Sets up a local development environment using conda

if ! command -v conda &> /dev/null
then
	echo "Please install anaconda and rerun this script"
	exit
fi

# Clone the repo
git clone -b develop https://github.com/WisconsinAutonomous/WisconsinAutonomous.github.io.git
cd WisconsinAutonomous.github.io

# Create the conda environment
conda env create -f environment.yml

# Patch gem/ruby/bundle
cd $(gem environment gemdir)
cd ../../$(basename $PWD)/$(gem environment platform | sed -e 's/.*://')
mv rbconfig.rb rbconfig.rb.bu
perl -pe 's/\/\S*?\/_build_env\/bin\///g' rbconfig.rb.bu > rbconfig.rb
gem install bundler jekyll

# Install packages for this repo
bundle
