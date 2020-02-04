# robotControl

This repository contains the code for the uStepper robotarm Rev 4. The repository is currently not structured as a regular arduino library, so installation using the arduino library manager is not possible yet, but it will be in the future. Until then, this repository should be downloaded to a folder, of your choosing, on your computer.

## Flashing of the uStepper S boards

The "usteppersrobotarm.ino" sketch contains the code for the 3 uStepper S boards on the uStepper arm. This sketch requires the newest version of the uStepper S library to be installed

### Flashing steps

    - Open the "usteppersrobotarm.ino" sketch in Arduino IDE
    - Go to "Tools->Boards" and choose "uStepper S"
    - Attach USB cable and power supply to the uStepper S board you want to flash
    - Turn on power supply and press "upload" button in Arduino IDE
    - Wait for flashing to finish
    - Repeat for the other uStepper S boards if nessecary

### Instructions for updating the WiFi hosted GUI

    - Download the binaries (two files) in the WiFi binaries folder
    - Follow the steps shown in this video: https://youtu.be/N0H2Ssl1WzI

<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" /></a><br /><span xmlns:dct="http://purl.org/dc/terms/" property="dct:title">uStepper & uStepper Robot Arm 4</span> by <a xmlns:cc="http://creativecommons.org/ns#" href="www.ustepper.com" property="cc:attributionName" rel="cc:attributionURL">ON Development</a> is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License</a>.
