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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="DriverControl: Teleop POV", group="Crushy")
//@Disabled
public class DriverControl extends LinearOpMode {

    /* Declare OpMode members. */
    CrushyHardware robot = new CrushyHardware();   // Use a Crushy's hardware
                                                               // could also use HardwarePushbotMatrix class.
    //double          clawOffset      = 0;                       // Servo mid position
    //final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

    @Override
    public void runOpMode() {
        double left;
        double right;
        double max;
        double leftPower;
        double rightPower;
        double speedControl = 0.5;
        boolean lbPressed = false;
        boolean rbPressed = false;
        double collectorPower = -1.0;
        boolean bPressed = false;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.gyroSensor.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && robot.gyroSensor.isCalibrating())  {
            sleep(50);
            idle();
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.left_bumper)
            {
                if(!lbPressed)
                {
                    if(speedControl < .95)
                    {
                        speedControl = speedControl + 0.1;
                    }
                    lbPressed = true;
                }
            }
            else
            {
                lbPressed = false;
            }

            if (gamepad1.right_bumper)
            {
                if(!rbPressed)
                {
                    if(speedControl > 0.05)
                    {
                        speedControl = speedControl - 0.1;
                    }
                    rbPressed = true;
                }
            }
            else
            {
                rbPressed = false;
            }


            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            left = -gamepad1.left_stick_y;
            right = gamepad1.right_stick_x;

            leftPower = left;
            rightPower = left;

            leftPower = leftPower + right;
            rightPower = rightPower - right;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0)
            {
                leftPower /= max;
                rightPower /= max;
            }
            robot.leftFront.setPower(leftPower * speedControl);
            robot.leftBack.setPower(leftPower * speedControl);
            robot.rightFront.setPower(rightPower * speedControl);
            robot.rightBack.setPower(rightPower * speedControl);

            if (gamepad2.b)
            {
                if (!bPressed)
                {
                    collectorPower = collectorPower * -1.0;

                    if (robot.particleCollector.getPower() != 0.0)
                    {
                        robot.particleCollector.setPower(collectorPower);
                    }

                    bPressed = true;
                }
            }
            else
            {
                bPressed = false;
            }

            if (gamepad2.left_trigger > 0)
            {
                robot.particleCollector.setPower(collectorPower);
            }

            if (gamepad2.left_bumper)
            {
                robot.particleCollector.setPower(0);
            }

            if (gamepad2.right_trigger > 0)
            {
                robot.leftShooter.setPower(1.0);
                robot.rightShooter.setPower(1.0);
            }

            if (gamepad2.right_bumper)
            {
                robot.leftShooter.setPower(0);
                robot.rightShooter.setPower(0);
            }

            if (gamepad2.left_stick_y < -0.1)
            {
                robot.leftServo.setPosition(0.0);
                robot.rightServo.setPosition(1.0);
            }
            else if (gamepad2.left_stick_y > 0.1)
            {
                robot.leftServo.setPosition(1.0);
                robot.rightServo.setPosition(0.0);
            }
            else
            {
                robot.leftServo.setPosition(0.5);
                robot.rightServo.setPosition(0.5);
            }

            int zValue = robot.gyroSensor.getIntegratedZValue();
            // Use gamepad left & right Bumpers to open and close the claw
            //if (gamepad1.right_bumper)
            //    clawOffset += CLAW_SPEED;
            //else if (gamepad1.left_bumper)
            //   clawOffset -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            //clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            //robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
            //robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);

            // Use gamepad buttons to move arm up (Y) and down (A)
            //if (gamepad1.y)
            //    robot.armMotor.setPower(robot.ARM_UP_POWER);
            //else if (gamepad1.a)
            //    robot.armMotor.setPower(robot.ARM_DOWN_POWER);
            //else
            //    robot.armMotor.setPower(0.0);

            // Send telemetry message to signify robot running;
            //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("speed",  "%.2f", speedControl);
            telemetry.addData("zValue", zValue);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }

    public void turnRobot(int direction)
    {
        int current = robot.gyroSensor.getHeading();

    }
}
