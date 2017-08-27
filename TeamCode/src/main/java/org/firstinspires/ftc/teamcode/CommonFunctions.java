package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
    double shooterPower = 0.0;

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

    public void RampUpShooter () {
        shooterPower = Range.clip(shooterPower + 0.01, 0, 0.7);
        robot.setShooterPower(shooterPower);
    }

    public void RampDownShooter () {
        robot.leftShooter.setPower(0.0);
        robot.rightShooter.setPower(0.0);
    }

    public void ShootBall () {
        robot.particleCollector.setPower(1.0);
        runtime.reset();
        while (runtime.seconds() < .5) {}
        robot.particleCollector.setPower(0.0);
    }
}