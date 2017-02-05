package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by CrushBots for the 2016-2017 FTC season
 */

@TeleOp(name="Driver TeleOp", group="CrushBots")
//@Disabled
public class DriverTeleOp extends CommonFunctions {

    /* Declare members. */
    private double speedControl = 0.5;

    private static final double shooterChangePower = 0.05;
    private double shooterPower = 0.0;
    private boolean shooterStarted = false;
    private boolean rightBumperIsPressed = false;

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        double left;
        double right;
        double max;
        double leftPower;
        double rightPower;

        /*
         * Left Bumper - Increase Speed
         */
        if (gamepad1.left_bumper) {
            speedControl = 0.95;
        } else {
            speedControl = 0.5;
        }

        /*
         * Right Bumper - Decrease Speed
         */
        if (gamepad1.right_bumper) {
                speedControl = 0.15;
        } else {
            speedControl = 0.5;
        }

        /*
         * Left Joy Stick - Forward / Reverse
         * Right Joy Stick - Left / Right
         */
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
        robot.setDrivePower((leftPower * speedControl), (rightPower * speedControl));

        /*
         * Left Bumper - Collector
         */
        if (gamepad2.left_bumper)
        {
            robot.particleCollector.setPower(1.0);
        }
        else
        {
            robot.particleCollector.setPower(0);
        }

        /*
         * Right Bumper - Shooter        // Step 10:  Pull Beacon Arms In
        robot.BeaconArmsServo.setPosition(1.0);
        robot.rightServo.setPosition(0.0);

         */
        if (!rightBumperIsPressed && gamepad2.right_bumper)
        {
            // Right Bumper is pressed - start shooter
            shooterStarted = !shooterStarted;
            rightBumperIsPressed = true;
        }
        else if (rightBumperIsPressed && !gamepad2.right_bumper)
        {
            // Right Bumper is released - stop shooter
            rightBumperIsPressed = false;
        }

        if (shooterStarted)
        {
            // Ramp Up the Shooter
            shooterPower = Range.clip(shooterPower + shooterChangePower, 0, 1.0);
            robot.leftShooter.setPower(shooterPower);
            robot.rightShooter.setPower(shooterPower);
        }
        else
        {
            // Ramp Down the Shooter
            shooterPower = Range.clip(shooterPower - shooterChangePower, 0, 1.0);
            robot.leftShooter.setPower(shooterPower);
            robot.rightShooter.setPower(shooterPower);
        }

        if (gamepad2.b)
        {
            robot.leftShooter.setPower(0);
            robot.rightShooter.setPower(0);
        }

        /*
         * Left Joy Stick - Move Beacon Arms
         */
        if (gamepad2.left_stick_y < -0.1)
        {
            // Move arms out
            robot.BeaconArmsServo.setPosition(0.0);
        }
        else if (gamepad2.left_stick_y > 0.1)
        {
            // Move arms in
            robot.BeaconArmsServo.setPosition(1.0);
        }

        /*
         * Right Joy Stick - Side Beacon Arms
         */
        if (gamepad2.right_stick_y > 0.1)
        {
            //robot.sideBeacon.setPower(0.2);
        }
        else if (gamepad2.right_stick_y < -0.1)
        {
            //robot.sideBeacon.setPower(-0.2);
        }
        else
        {
            //robot.sideBeacon.setPower(0.0);
        }

        /*
         * Right Bumper - Shooter
         */
        if (!rightBumperIsPressed && gamepad2.right_bumper)
        {
            // Right Bumper is pressed.
            shooterStarted = !shooterStarted;
            rightBumperIsPressed = true;
        }
        else if (rightBumperIsPressed && !gamepad2.right_bumper)
        {
            // Right Bumper is released.
            rightBumperIsPressed = false;
        }

        if (shooterStarted)
        {
            shooterPower = Range.clip(shooterPower + shooterChangePower, 0, 0.55);
            robot.leftShooter.setPower(shooterPower);
            robot.rightShooter.setPower(shooterPower);
        }
        else
        {
            shooterPower = Range.clip(shooterPower - shooterChangePower, 0, 1.0);
            robot.leftShooter.setPower(shooterPower);
            robot.rightShooter.setPower(shooterPower);
        }
    }
}