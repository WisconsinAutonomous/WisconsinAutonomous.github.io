#!/bin/bash

# Sets up a local development environment using conda

check_command() {
	cmd=$1
	if ! command -v $cmd &> /dev/null
	then
		exit_error "Please install $cmd and rerun this script"
	fi
}

exit_error() {
	msg=$1
	if [ -z "$1" ]; then
		msg="Unknown Error"
	fi
	echo -e "\nScript exited with error: $msg"
	exit 1
}

exit_okay() {
	msg=$1
	if [ -z "$msg" ]; then
		msg="OK"
	fi
	echo -e "\nScript exitted: $msg"
	exit 0
}

ask_okay() {
	msg=$1
	if [ -z "$msg" ]; then
		exit_error "Pass a message to the ask_okay function."
	fi

	read -p "$msg ([y]/n)? " CONT
	case ${CONT:0:1} in
	y | Y | "")
		return 0
		;;
	*)
		return -1
		;;
	esac
}

ask_response() {
	msg=$1
	if [ -z "$msg" ]; then
		exit_error "Pass a message to the ask_okay function."
	fi

	read -p "$msg " RESP
	echo $RESP
}

check_command conda
check_command curl
check_command git

# Verify the script was run on purpose
if ask_okay "Create new conda env"; then
	CREATE_NEW_CONDA_ENV=1
elif ask_okay "Update conda env"; then
	UPDATE_CONDA_ENV=1
else
	exit_okay
fi

# Clone the repo
if ask_okay "Clone WisconsinAutonomous.github.io in $PWD"; then
	git clone -b develop https://github.com/WisconsinAutonomous/WisconsinAutonomous.github.io.git
	cd WisconsinAutonomous.github.io
elif [[ "$(basename $(git remote show -n origin | grep Fetch | cut -d: -f2-))" != "WisconsinAutonomous.github.io" ]]; then
	exit_error "Please WisconsinAutonomous.github.io or say yes to the previous question"
fi


# Get the environment.yml file from github
env_file=$(curl -fsSL https://raw.githubusercontent.com/WisconsinAutonomous/WisconsinAutonomous.github.io/master/environment.yml)

# Check the name
name="wa"
if ! ask_okay "Environment name '$name' okay"; then
	name=$(ask_response "Environment name :: ")
fi

# Create the conda environment from the retrieved environment.yml file
tmpfile=temp_env.yml
echo "$env_file" >>$tmpfile
if [ -n "$CREATE_NEW_CONDA_ENV" ]; then
	conda env create --name=$name -f=$tmpfile
elif [ -n "$UPDATE_CONDA_ENV" ]; then
	conda env update --name=$name -f=$tmpfile
fi
rm -f $tmpfile

# Patch gem/ruby/bundle
cd $(gem environment gemdir)
cd ../../$(basename $PWD)/$(gem environment platform | sed -e 's/.*://')
mv rbconfig.rb rbconfig.rb.bu
perl -pe 's/\/\S*?\/_build_env\/bin\///g' rbconfig.rb.bu > rbconfig.rb
gem install bundler jekyll

# Install packages for this repo
bundle
