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
Enter "1" for calibration, and do NOT touch or jostle the electronics while the calibration process is underway (until the main menu reappears).

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

**DATA FROM SENSORS SEEMS INCORRECT/CORRUPTED**
1. Return to the Main Menu and choose option 1 for recalibration.
2. If that does not work, reset the board by pressing the button, and see if the issue is resolved.
3. Confirm that the affected sensor is properly connected to the Arduino. If it is not, redo its connections.
4. If the problem continues to persist, there is likely a hardware issue. Replace the affected sensor.

**CODE IS STUCK/NOT PROCEEDING AS EXPECTED**
1. IMMEDIATELY feel the Arduino to confirm that it is not hot. If it is, *DISCONNECT THE BATTERY AND ANY COMPUTERS/OTHER DEVICES ASAP.*
2. Reset the board by pressing the button.
3. If the problem persists, reflash the code to the board.
4. Check that the ICM20948 and DPS310 are properly connected to the board.

**CODE IS NOT WRITING TO SERIAL**
1. Confirm that the baud rate on your device is set to 115200.
2. Ensure that your device is properly connected to the Arduino.
3. Ensure that your device is detecting that the board is connected (Arduino UNO via usbmodem).
4. Reset the code by pressing the button on the board.
5. Reflash the code to the Arduino.
6. Update the drivers on your device.
7. Try a different device.

**AIRBRAKES ARE NOT MOVING**
1. Listen to the motor and ensure it is spinning. If it is not, proceed to step 5.
2. Make sure that the shaft has not decoupled from the motor by trying to manually turn the shaft.
3. Make sure that the actuation rods are connected to the motor shaft with the hex screws.
4. Make sure that the boattail is not obscuring the airbrakes.
5. IF MOTOR IS NOT SPINNING: Ensure that the motor driver, motor, and board are all properly connected.
6. If they are, disconnect the motor from the motor driver, and use a 12V power supply to try to drive the motor directly.
7. If the motor still does not spin, it is broken. Replace the motor.
8. If the motor does spin when connected directly to power, the motor driver is broken. Replace the motor driver.
