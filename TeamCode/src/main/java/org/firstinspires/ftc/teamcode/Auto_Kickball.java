package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by CrushBots for the 2016-2017 FTC season
 */

@Autonomous(name="KickBall", group="Autonomous")
//@Disabled
public class Auto_Kickball extends Auto_CommonFunctions {

    /* Declare OpMode members. */
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

        // Step 1: Drive forward
        robot.leftFront.setPower(0.5);
        robot.leftBack.setPower(0.5);
        robot.rightFront.setPower(0.5);
        robot.rightBack.setPower(0.5);
        sleep(1000);
        robot.leftFront.setPower(0.0);
        robot.leftBack.setPower(0.0);
        robot.rightFront.setPower(0.0);
        robot.rightBack.setPower(0.0);


        // Step 2: Shoot Balls
        ShootBalls();

        // Step 3: Drive forward, kick ball and park
        robot.leftFront.setPower(0.5);
        robot.leftBack.setPower(0.5);
        robot.rightFront.setPower(0.5);
        robot.rightBack.setPower(0.5);
        sleep(700);
        robot.leftFront.setPower(0.0);
        robot.leftBack.setPower(0.0);
        robot.rightFront.setPower(0.0);
        robot.rightBack.setPower(0.0);
    }
}
