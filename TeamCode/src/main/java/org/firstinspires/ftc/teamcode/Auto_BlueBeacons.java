package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by CrushBots for the 2016-2017 FTC season
 */

@Autonomous(name="BlueBeacons", group="Autonomous")
//@Disabled
public class Auto_BlueBeacons extends CommonFunctions {

  int state = 0;

  @Override
  public void loop() {

    switch (state) {
      case 0:   // Drive forward
        DriveForwardTime();

        state++;
        break;

      case 1:   // Shoot balls
        ShootBalls();

        state++;
        break;

      case 2:   // Turn right towards Beacons
        DriveForwardTime();

        state++;
        break;

      case 3:   // Drive forward until Beacon line
        DriveForwardTime();

        state++;
        break;

      case 4:   // Process Beacon
        DriveForwardTime();

        state++;
        break;

      case 5:   // Drive forward until Beacon line
        DriveForwardTime();

        state++;
        break;

      case 6:   // Process Beacon
        DriveForwardTime();

        state++;
        break;

      case 7:   // Turn to ramp
        DriveForwardTime();

        state++;
        break;

      case 8:   // Drive onto ramp
        DriveForwardTime();

        state++;
        break;
    }
  }
}