# gdkGBA_cg50
port of gdkChan's gba emulator to fx-cg50

## The Goal

The goal is to port the emulator to a functional state. The second aim will be to optimize it as much as possible, to see if we can attain a playable state in a future implementation.

This precise implementation will most likely NOT be a good base for a playable emulator on the fx-cg50, because of the unoptimized nature of gdkGBA that was not made to run on a calculator to begin with.

## Current State

the project compiles using fxSDK and Gint, it starts up all the way until ROM load where it crashes. Most likely due to loading the entire damn ROM at once.
Latest commit implements memory chunk loading, but i couldn't test it just yet (my calculator's USB cable gave up and i am buying a new one).
