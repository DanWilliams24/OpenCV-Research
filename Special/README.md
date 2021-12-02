# OpenCV-Research
This is a mini project proposed by a friend in 2018. Create a program that locks your computer if it detects that someone is in front of it.
The idea being that in enviornments where you want to keep your computer running but still want to preserve your privacy, you can have the computer autonomously lock and screenshot someone snooping on your device without permission. 

## Notes:
Implements a pretrained Caffe model acquired online for the detection of people. Somewhat overkill as what we really need is a strict person detector versus an multi object classifier, but it was what I had on hand in the 1 night I spent setting up the code and bash scripts.

## Up Next: 
Ideally, the point of this mini-project was to prevent outsiders from snooping, but what if you want to give specific people (including yourself) access to your machine (like a whitelist). So, to take this a step further, I would like to refine the detector to only detect people and have the ability to recognize my face specifically. This way I can bypass the scanner and my program  can stop repeatedly locking me out thinking I am the threat lol.
