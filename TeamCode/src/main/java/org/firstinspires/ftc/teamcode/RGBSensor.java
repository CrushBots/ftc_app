package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by CrushBots on 8/30/2016.
 */
public class RGBSensor extends OpMode {
    ColorSensor colorSensor;

    public void init() {
        colorSensor = hardwareMap.colorSensor.get("rgbSensor");
    }
    public void loop() {
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
    }

}
