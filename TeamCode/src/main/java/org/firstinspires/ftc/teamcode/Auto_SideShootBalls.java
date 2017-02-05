package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by CrushBots for the 2016-2017 FTC season
 */

@Autonomous(name="Side Shoot Balls", group="Autonomous")
//@Disabled
public class Auto_SideShootBalls extends CommonFunctions {

  int state = 0;

  @Override
  public void loop() {

    switch (state) {
      case 0:   // Drive forward
        DriveForwardTime();

        state++;
        break;

      case 1:   // Shoot Balls
        ShootBalls();

        state++;
        break;
    }
  }
}