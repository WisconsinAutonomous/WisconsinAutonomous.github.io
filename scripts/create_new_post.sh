#!/bin/bash

# ---------------------
# Config initialization
# ---------------------
join() {
	str=""
	local counter=0
	for c in "$@"; do
		counter=$((counter + 1))
		[ "$counter" == "1" ] && str="$c" && continue
		str="$str, $c"
	done
	echo "$str"
}

[ ! -x "$(command -v git)" ] && echo "Please install git before proceeding" && exit 1
[ ! "$(git rev-parse --is-inside-work-tree 2>/dev/null)" ] || [ ! "$(basename `git rev-parse --show-toplevel`)" = "WisconsinAutonomous.github.io" ] && echo "Please run this command from within the WisconsinAutonomous.github.io repo." && exit 1
TOP_LEVEL=$(git rev-parse --show-toplevel)

categories=($(ls $TOP_LEVEL/_posts))


# ----------------------------
# Parse command line arguments
# ----------------------------
source "$TOP_LEVEL/scripts/thirdparty/optparse.sh"

# Define options
# Required
optparse.define short=p long=post desc="The post name (used to create the file)." variable=POST
optparse.define short=t long=title desc="The title of the post (wrap in quotes)." variable=TITLE
optparse.define short=n long=name desc="Your name or the name of the poster (wrap in quotes)." variable=NAME
optparse.define short=e long=email desc="Your email ending in '@wisc.edu' (wrap in quotes)." variable=EMAIL
optparse.define short=c long=category desc="The category your post fits in. Must be one of the following: $(join ${categories[@]})." variable=CATEGORY

# Parse
source $( optparse.build )

# Check the inputs
print_help() {
	set -- "--help"
	source $( optparse.build )
	exit 1
}
check_required() {
	if [ "${!1}" == "" ]; then
		lower=${1,,}
		echo "ERROR: -${lower::1},--$lower [$1] is required."
		echo
		print_help
	fi
}
check_in_array() {
	if [[ ! " ${categories[@]} " =~ " ${!1} " ]]; then
		echo "ERROR: $1 was '${!1}'"
		echo "ERROR: $1 must be one of the following: ${categories[*]}."
		echo
		print_help
	fi
}
check_required POST
check_required TITLE
check_required NAME
check_required EMAIL
check_required CATEGORY

check_in_array CATEGORY


# ---------------
# Create the post
# ---------------
get_defaults() {
	IFS=$'\n' read -d '' -r -a $2 < $LOCATION.$1
}

DATE=$(date +"%F")
TIME=$(date +"%H:%M:%S %z")
FILE="$DATE-$POST.md"
LOCATION="$TOP_LEVEL/_posts/$CATEGORY/"

get_defaults tags default_tags
get_defaults categories default_categories

boilerplate="---
title: $TITLE
author: $NAME
date: $DATE $TIME
categories: [$(join ${default_categories[@]})]
tags: [$(join ${default_tags[@]})]
---

This is a boilerplate post created with the 'create_new_post.sh' script.

## Setup Guide

_Set up guide_

## Support

Contact [$NAME](mailto:$EMAIL) for any questions or concerns regarding the contents of this post.

## See Also

Stay up to date with our technical info by following our [blog](https://wa.wisc.edu/blog).

Follow us on [Facebook](https://www.facebook.com/wisconsinautonomous/), [Instagram](https://www.instagram.com/wisconsinautonomous/), and [LinkedIn](https://www.linkedin.com/company/wisconsin-autonomous/about/)!

![WA Logo](/assets/img/logos/wa-white.png){: .left height="100"}
![Wisconsin Crest](/assets/img/logos/uw-crest.png){: .right height="100"}
"

echo "Creating new file '$FILE' in $LOCATION"
sleep 1 
echo "$boilerplate" >> $LOCATION$FILE
echo "Done."
