package org.firstinspires.ftc.teamcode;

/**
 * Created by CrushBots on 8/10/2016.
 */
public class TestBoard extends Hardware{

    @Override public void loop() {

        drive();

        positionArm();

        touchSensorPressed();

        measureDistance();

        //measureColor();
    }
}