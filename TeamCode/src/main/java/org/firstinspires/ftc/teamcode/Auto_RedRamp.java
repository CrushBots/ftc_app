package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by CrushBots for the 2016-2017 FTC season
 */

@Autonomous(name="Red Ramp", group="Autonomous")
//@Disabled
public class Auto_RedRamp extends CommonFunctions {

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

      case 2:   // Turn left towards Ramp
        DriveForwardTime();

        state++;
        break;

      case 3:   // Drive forward onto the ramp
        DriveForwardTime();

        state++;
        break;
    }
  }
}