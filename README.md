# AirbrakesTestCode23-24
Code for 23-24 airbrakes test rocket


Units:

Time: Microseconds after system start

Acceleration: Gs

Gyroscopic Data: Degrees

Magnetometer Data: microTeslas

All Temperature Data: Degrees Celsius

Pressure: hectoPascals

Altitude: Meters


<h1>General Usage:</h2>

<h2>UPON STARTUP:</h2>

The following menu will appear:

<i>
Please enter the number that corresponds with the menu option you would like to choose.<br/>
1. Calibrate<br/>
2. Test Motor<br/>
3. Test Sensors<br/>
4. Pad-Idle Mode<br/>
</i>

<br/>
Enter "1" for calibration, and do NOT touch or jostle the electronics while the calibration process is underway (unitl the main menu reappears).

The following message will appear:

*Calibrating IMU...*<br/>
*Please enter the current pressure in hPa.*

Enter the pressure in hectoPascals, a floating point number greater than 0. 

*(NOTE: If possible, try not to use readings from a weather app, as this is approximated based on GPS location. Instead, use barometric readings from a phone or other device with a barometer.)*

You may now choose another menu option depending on which function you want to use. If at any time sensor readings seem inaccurate, recalibrate.


<h1>Menu Options:</h1>

<h2>TEST MOTOR:</h2>

**NOTE: ALL DEPLOYMENT FUNCTIONS WILL FIRST RETRACT THE AIRBRAKES TO AVOID OVER EXTENSION AND MOTOR DAMAGE.**

The following menu will appear:

<i>
Please enter the number that corresponds with the menu option you would like to choose.<br/>
1. User Input Mode<br/>
2. One Cycle<br/>
3. Full Deploy<br/>
4. Retract<br/>
5. Return to Main Menu<br/>
</i>

Enter the number corresponding to the option you want to choose.

<h3>User Input Mode:</h3>

**PLEASE NOTE: THIS IS LOWER PRIORITY AND IS THEREFORE NOT YET IMPLEMENTED.**

This will prompt the user for an angle (from fully retracted) to deploy the airbrakes to.

<h3>One Cycle:</h3>

This will deploy the airbrakes, wait 0.5 seconds, and retract the airbrakes.

<h3>Full Deploy:</h3>

This will deploy the airbrakes to maximum extension.

<h3>Retract Airbrakes:</h3>

This will retract the airbrakes to their stowed position.

<h2>Test Sensors</h2>
This will provide a readout of every value that each sensor (DPS310 barometer and ICM20948 intertial measurement unit) is currently reading. This is useful to confirm that the sensors do not need to be recalibrated.

<h2>Pad Idle Mode</h2>

*This is the nominal use mode, and should ALWAYS be set to run before the rocket is launched. Once this mode is selected, only gentle movements should be used, as the IMU will be scanning for high accelerations to detect launch. Once this mode is selected, the only way to cancel it is to reset the board (press the button/reflash the code), or cut power to the board.*

This mode tests the airbrakes during flight and logs data. Upon choosing this mode, the code will continuously scan for high accelerations to detect launch, and begin logging data from all sensors. This logging will continue until after touchdown.

After selecting this mode, you may disconnect your device from the board and set the rocket up for launch.

Once launch is detected (net acceleration is above 3 Gs), the code will scan for burnout by detecting when acceleration falls below 4 Gs. 

*(NOTE: There is a time based lockout to prevent accidental airbrakes deployment during the boost stage, and burnout will not be detected if an acceleration drop occurs before 1 second after launch. There is also a failsafe to proceed with airbrakes deployment if burnout has not been detected more than 1.8 seconds after launch.)*

It will then wait one second, deploy the airbrakes, wait another second, and retract the airbrakes.

The IMU will then scan for an acceleration value between 0.7 and 1.3 Gs to detect touchdown. Touchdown will only be confirmed if 10 such consecutive readings are taken.

Once tounchdown is confirmed, it will wait one second, and then dump all of the data logged throughout the flight to the SD card.

Upon recovery of the rocket, IMMEDIATELY connect a computer with a serial input to the board. DO NOT DISCONNECT THE BATTERY OR RESET THE BOARD. Open a Serial Monitor, and ensure that it displays the following message:

*PLEASE ENTER 1234567890 ONCE YOU HAVE EJECTED THE SD CARD AND COMPLETED OPERATIONS.*

Remove the SD card from the SD card slot, and enter 1234567890 to confirm that you have done so. Pad Idle Mode will now complete.



<h1>TROUBLESHOOTING</h1>
