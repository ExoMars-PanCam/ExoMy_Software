# Notes for Dev

A brief summary of key points and things to fix.

## Walking legs

It's rather challenging not having any sort of knowledge of the position.

The wheels should be pointing backwards (away from the U-shaped bracket).

This then allows for the slow rise upon start-up. 

## Controller

Good link for setup of the controller. Should be distilled here:
https://pimylifeup.com/xbox-controllers-raspberry-pi/ 

`sudo jstest /dev/input/js0` is the best for testing controller in terminal.

## Testing in dev environment

`sh ./docker/run_exomy.sh -d`

## Getting to stand

Complication with not being able to know the status of the motors. So you always have to change
state to begin with. 

Operaiton is:
- Press select: To enter change walking state
- Select: LB to Sit, RB to Stand
- Press select again to exit change walking state
- Press start: To enable the drive motors
- Repeat above for sitting again.