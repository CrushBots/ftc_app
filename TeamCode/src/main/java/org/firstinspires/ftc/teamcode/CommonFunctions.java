package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by CrushBots for the 2016-2017 FTC season
 */

@TeleOp(name="CommonFunctions", group="Stuff")
@Disabled
public class CommonFunctions extends OpMode
{
    /* Declare OpMode members. */
    protected ElapsedTime runtime = new ElapsedTime();
    CrushyHardware robot = new CrushyHardware();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables.
        robot.init(hardwareMap);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void DriveForwardTime () {
        double speed = -0.5;

        robot.setDrivePower(speed, speed);

        runtime.reset();
        while (runtime.seconds() < 0.5) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.setDrivePower(0, 0);
    }

    public void RampUpShooter () {
        double shooterPower = 0;

        // Power was .52
        while (shooterPower < 0.03) {
            shooterPower = Range.clip(shooterPower + 0.01, 0, 0.03);
            robot.setShooterPower(shooterPower);
        }
        runtime.reset();
        while (runtime.seconds() < 3.0) {}
    }

    public void RampDownShooter () {
        double shooterPower = 0.55;

        while (shooterPower > 0) {
            shooterPower = Range.clip(shooterPower - 0.05, 0, 0.55);
            robot.setShooterPower(shooterPower);
        }
        runtime.reset();
        while (runtime.seconds() < 3.0) {}
    }

    public void ShootBall () {
        robot.particleCollector.setPower(1.0);
        runtime.reset();
        while (runtime.seconds() < .5) {}
        robot.particleCollector.setPower(0.0);
    }

    public void ShootBalls () {
        RampUpShooter();
        ShootBall();
        RampUpShooter();
        ShootBall();
        RampDownShooter();
    }

}