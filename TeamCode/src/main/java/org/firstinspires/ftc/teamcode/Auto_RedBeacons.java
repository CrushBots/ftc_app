package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by CrushBots for the 2016-2017 FTC season
 */

@Autonomous(name="Red Beacons", group="Autonomous")
//@Disabled
public class Auto_RedBeacons extends Auto_CommonFunctions {

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
        DriveInches(17);

        // Step 2: Shoot Balls
        ShootBalls();

        // Step 3: Turn left toward beacons
        turnLeft(12);

        // Step 4: Drive to beacons
        DriveInches(18);

        // Step 5: Turn right so parell with beacons
        turnRight(12);

        // Step 6: process beacon
        ProcessRedBeacon();

        // Step 7: Drive to next beacons
        DriveInches(5);

        // Step 8: process beacon
        ProcessRedBeacon();
    }
}
