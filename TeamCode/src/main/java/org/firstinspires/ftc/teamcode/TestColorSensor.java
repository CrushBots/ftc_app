package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by CrushBots for the 2016-2017 FTC season
 */

@TeleOp(name = "Test Color Sensor", group = "Test")
//@Disabled
public class TestColorSensor extends CommonFunctions {

  // hsvValues is an array that will hold the hue, saturation, and value information.
  private float hsv[] = {0F,0F,0F};

  @Override
  public void loop() {

    robot.leftBeaconColorSensor.enableLed(true);

    Color.RGBToHSV(robot.leftBeaconColorSensor.red() * 8, robot.leftBeaconColorSensor.green() * 8, robot.leftBeaconColorSensor.blue() * 8, hsv);

    telemetry.addData("Color", getColorName(hsv));

    telemetry.addData("Red", robot.leftBeaconColorSensor.red());
    telemetry.addData("Green", robot.leftBeaconColorSensor.green());
    telemetry.addData("Blue", robot.leftBeaconColorSensor.blue());

    telemetry.addData("Hue", hsv[0]);
    telemetry.addData("Saturation", hsv[1]);
    telemetry.addData("Value", hsv[2]);
  }

  public String getColorName(float[] hsv) {
    if ((hsv[0] < 30 || hsv[0] > 340) && hsv[1] > .2) return "Red";
    else if ((hsv[0] > 170 && hsv[0] < 260) && hsv[1] > .2) return "Blue";
    return "undefined";
  }
}