StopAndTail
===========

Source code for a PIC 12F675 controlled stop and tail lamp (for motorcycles or cars)


Operation:

Pressing the brake light 15 times (with no more than around 1 second between each press) will switch the unit into configuration mode, editing the tail light.

Whilst in configuration mode, each single press of the brake lever will increase the perceived brightness of the tail light.  When the tail light brightness reaches the brightness of the brake light, it will drop to the lowest allowed brightness.

Pressing the brake lever for around 5 seconds will cause the unit to flash twice and enter the editing of the brake light brightness.

Whilst editing the brake light, each press will increase the brightness to the maximum allowed brightness.  When the brightness reaches the maximum brightness, the brightness will be reset to one above the current tail light brightness.

Pressing the brake lever for around 5 seconds will cause the unit to flash once and return to editing of the tail light.

Pressing the brake lever for around 10 seconds will cause the unit to store the current values and exit configuration mode.

Turning the unit off (ie. with the ignition or disconnecting the battery) or leaving the unit idle for 30 seconds will cause the unit to exit configuration mode and 'forget' the current settings.
