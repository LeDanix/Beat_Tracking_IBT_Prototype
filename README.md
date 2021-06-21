The repository contains the source code of the IBT Beat-Tracking program modified by Daniel Saiz for be able to work associated with a ESP-32 microcontroller, and the LED prototype code.
The aim to this proyect is build a functional prototype that allows users to improve theirs beat tracking abilities.

This proyect only work with a electronic device. His electronic esquematic is on my easyEDA repository: 
This device is made by a Matrix of LED that allows to visualize each beat that IBT program detects. Works in real time, catching audio from the computer mic.
If you want to build this device, the code that allows his functions is on the BeatIndicatorMatrix folder of this repository. For more information about flashing a ESP-32, visit the espressif documentation.

¡¡¡Only works on Windows!!!

Instructions

1º. Open command window

2º. Execute ibt.exe with the next instruction:   ibt -p ''COM-PORT'' -mic  where ''COM-PORT'' is the number of the COM port that the electronic device is conected. To know this, you can write the instruction 'mode' into your command window to see what COM ports are enables. marsyas.dll is compleatly necesary to run ibt.exe.

3º. Play any music

The algorithm works really well with electronic music or some genres with loud beats but his performance is not so good in genres like clasical music or jazz.