package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by CrushBots for the 2016-2017 FTC season
 */

@TeleOp(name = "Test Encoders", group = "Test")
//@Disabled
public class TestEncoders extends CommonFunctions {

  @Override
  public void loop() {

    RampUpShooter();

    ShootBall();

    telemetry.addData("Left Shooter Encoder Position", robot.leftShooter.getCurrentPosition());
    telemetry.addData("Right Shooter Encoder Position", robot.rightShooter.getCurrentPosition());
  }
}