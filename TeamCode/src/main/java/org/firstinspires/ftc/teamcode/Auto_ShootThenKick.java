package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by CrushBots for the 2016-2017 FTC season
 */

@Autonomous(name="Shoot Then Kick", group="Autonomous")
//@Disabled
public class Auto_ShootThenKick extends CommonFunctions {

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

      case 2:   // Drive forward and kick balls
        DriveForwardTime();

        state++;
        break;
    }
  }
}