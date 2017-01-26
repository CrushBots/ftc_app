/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "sensor_light")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on asnd off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="BlueJustShootThenRamp", group="Crushy")
//@Disabled
public class BlueJustShootThenRamp extends LinearOpMode {

    /* Declare members. */
    CrushyHardware robot = new CrushyHardware();   // Use a Crushy's hardware
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {

        }

        // Step 1:  Put beacon arms out
        robot.leftServo.setPosition(0.0);
        robot.rightServo.setPosition(1.0);

        // Step 2:  Drive forward
        double speed = -0.5;
        robot.leftFront.setPower(speed);
        robot.leftShooter.

        robot.leftBack.setPower(speed);
        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

        // Step 3:  Ramp up Fly Wheel
        double shooterPower = 0;
        while (shooterPower < 0.52) {
            shooterPower = Range.clip(shooterPower + 0.05, 0, 0.52);
            robot.leftShooter.setPower(shooterPower);
            robot.rightShooter.setPower(shooterPower);
        }
        sleep(3000);

        // Step 4:  Run Collector - Ball 1
        robot.particleCollector.setPower(1.0);
        sleep(500);
        robot.particleCollector.setPower(0.0);

        // Step 5:  Ramp up Fly Wheel
        while (shooterPower < 0.52) {
            shooterPower = Range.clip(shooterPower + 0.05, 0, 0.52);
            robot.leftShooter.setPower(shooterPower);
            robot.rightShooter.setPower(shooterPower);
        }
        sleep(2000);

        // Step 6:  Run Collector - Ball 2
        robot.particleCollector.setPower(1.0);
        sleep(1000);
        robot.particleCollector.setPower(0.0);

        // Step 7:  Ramp Down Fly Wheel
        while (shooterPower > 0) {
            shooterPower = Range.clip(shooterPower - 0.05, 0, 0.55);
            robot.leftShooter.setPower(shooterPower);
            robot.rightShooter.setPower(shooterPower);
        }

        // Step 8:  Pivot Turn Right
        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);
        sleep(1300);
        robot.rightFront.setPower(0.0);
        robot.rightBack.setPower(0.0);

        // Step 9:  Drive Forward
        robot.leftFront.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.rightFront.setPower(speed);
        robot.rightBack.setPower(speed);
        sleep(1200);
        robot.leftFront.setPower(0.0);
        robot.leftBack.setPower(0.0);
        robot.rightFront.setPower(0.0);
        robot.rightBack.setPower(0.0);

        // Step 10:  Pull Beacon Arms In
        robot.leftServo.setPosition(1.0);
        robot.rightServo.setPosition(0.0);
    }
}