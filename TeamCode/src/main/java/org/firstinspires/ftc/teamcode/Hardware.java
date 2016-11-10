package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by CrushBots on 8/10/2016.
 */
public class Hardware extends OpMode {

    final static double SERVO_MIN_RANGE = 0.00;
    final static double SERVO_MAX_RANGE = 1;

    public DcMotor motorLeft    = null;
    public DcMotor motorRight   = null;
    public Servo servo          = null;
    public TouchSensor touchSensor     = null;
    public OpticalDistanceSensor distanceSensor     = null;
    public LightSensor lightSensor     = null;

    public double armPosition;
    public double armDelta = 0.1;
    public boolean ignoreButtonA = false;
    public boolean ignoreButtonY = false;

    private boolean v_warning_generated = false;
    private String v_warning_message;

    @Override public void init ()
    {

        // The variable below is used to provide telemetry data to a class user.
        v_warning_generated = false;
        v_warning_message = "Can't map; ";


        // Connect the drive wheel motors.
        try
        {
            motorLeft = hardwareMap.dcMotor.get ("motorLeft");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("left_drive");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            motorLeft = null;
        }

        try
        {
            motorRight = hardwareMap.dcMotor.get ("motorRight");
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("left_drive");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            motorRight = null;
        }

        try
        {
            servo = hardwareMap.servo.get ("servo");
            armPosition = 0.5;
            servo.setPosition(armPosition);
        }
        catch (Exception p_exeception)
        {
            m_warning_message ("servo");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            servo = null;
        }

        try
        {
            touchSensor = hardwareMap.touchSensor.get ("touchSensor");

        }
        catch (Exception p_exeception)
        {
            m_warning_message ("touchSensor");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            touchSensor = null;
        }

        try
        {
            distanceSensor = hardwareMap.opticalDistanceSensor.get ("distanceSensor");

        }
        catch (Exception p_exeception)
        {
            m_warning_message ("distanceSensor");
            DbgLog.msg (p_exeception.getLocalizedMessage ());

            distanceSensor = null;
        }

        try
        {
            lightSensor = hardwareMap.lightSensor.get ("colorSensor");

        }
        catch (Exception p_exeception)
        {
            m_warning_message ("lightSensor");
            DbgLog.msg (p_exeception.getLocalizedMessage());

            lightSensor = null;
        }

    }

    @Override public void start ()
    {
    }

    @Override public void loop ()
    {
    }

    @Override public void stop ()
    {
    }

    //--------------------------------------------------------------------------
    //
    // set_drive_power
    //

    void set_drive_power (float p_left_power, float p_right_power)
    {
        if (motorLeft != null)
        {
            motorLeft.setPower (p_left_power);
        }
        if (motorRight != null)
        {
            motorRight.setPower (p_right_power);
        }
    }

    //--------------------------------------------------------------------------
    //
    // set_drive_power
    //
    // left_stick_y ranges from -1 to 1, where -1 is full up and 1 is full down.
    //

    void drive()
    {
        float leftpower = 0;
        float rightpower = 0;

        float righttrigger = gamepad1.right_trigger;

        if (gamepad1.left_stick_y < 0) {
            leftpower = righttrigger;
        }
        else if (gamepad1.left_stick_y > 0)  {
            leftpower = -1 * righttrigger;
        }

        if (gamepad1.right_stick_y < 0) {
            rightpower = righttrigger;
        }
        else if (gamepad1.right_stick_y > 0){
            rightpower = -1 * righttrigger;
        }

        set_drive_power(leftpower, rightpower);

    }

    //--------------------------------------------------------------------------
    //
    // positionArm
    //

    void positionArm ()
    {
        if (gamepad1.a && !ignoreButtonA) {
            ignoreButtonA = true;
            armPosition += armDelta;
        }
        else
        {
            if (!gamepad1.a) {
                ignoreButtonA = false;
            }
        }

        if (gamepad1.y && !ignoreButtonY) {
            ignoreButtonY = true;
            armPosition -= armDelta;
        }
        else
        {
            if (!gamepad1.y) {
                ignoreButtonY = false;
            }
        }

        armPosition = Range.clip(armPosition, SERVO_MIN_RANGE, SERVO_MAX_RANGE);

        servo.setPosition(armPosition);

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("arm", "arm:  " + String.format("%.2f", armPosition));
    }

    void touchSensorPressed(){
        if (touchSensor.isPressed()){
            motorLeft.setDirection(DcMotor.Direction.REVERSE);
        }
        else {
            motorLeft.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    void measureDistance(){
        double  distance= distanceSensor.getLightDetected();

        telemetry.addData("Distance = ", distance);
    }

    void measureColor(){
        telemetry.addData("Color Sensor", "Hi!");
        double  distance = lightSensor.getLightDetected();

        telemetry.addData("NXT Light Sensor = ", distance);
    }

    //--------------------------------------------------------------------------
    //
    // a_warning_generated
    //
    boolean a_warning_generated ()

    {
        return v_warning_generated;

    } // a_warning_generated

    //--------------------------------------------------------------------------
    //
    // a_warning_message
    //
    String a_warning_message ()

    {
        return v_warning_message;

    }

    //--------------------------------------------------------------------------
    //
    // m_warning_message
    //
    void m_warning_message (String p_exception_message)

    {
        if (v_warning_generated)
        {
            v_warning_message += ", ";
        }
        v_warning_generated = true;
        v_warning_message += p_exception_message;

    }
}