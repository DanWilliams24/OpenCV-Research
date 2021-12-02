#!/bin/bash

python3 personDetector.py
python3 shirtDetector.py
VALID=$( cat isValidSession.txt )


if [ "$VALID" == "False" ]; then
	/System/Library/CoreServices/Menu\ Extras/user.menu/Contents/Resources/CGSession -suspend
fi



