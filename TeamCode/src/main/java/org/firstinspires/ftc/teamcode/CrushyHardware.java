 package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static java.lang.Thread.sleep;

 /**
  * Created by CrushBots for the 2016-2017 FTC season
  */

 public class CrushyHardware
{
    /* Public members. */
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor particleCollector = null;
    public DcMotor leftShooter = null;
    public DcMotor rightShooter = null;
    public Servo BeaconArmsServo = null;
    public ModernRoboticsI2cGyro gyroSensor = null;
    public ColorSensor leftBeaconColorSensor = null;
    public ColorSensor rightBeaconColorSensor = null;
    public ColorSensor leftUnderColorSensor = null;
    public ColorSensor rightUnderColorSensor = null;

    /* Local members. */
    HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public CrushyHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
         hwMap = ahwMap;

        /**
         *  Define and Initialize Motors
         */
        leftFront = hwMap.dcMotor.get("leftFront");
        leftBack = hwMap.dcMotor.get("leftBack");
        rightFront = hwMap.dcMotor.get("rightFront");
        rightBack = hwMap.dcMotor.get("rightBack");
        particleCollector = hwMap.dcMotor.get("particleIntake");
        leftShooter = hwMap.dcMotor.get("leftShooter");
        rightShooter = hwMap.dcMotor.get("rightShooter");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        particleCollector.setDirection(DcMotor.Direction.REVERSE);
        leftShooter.setDirection(DcMotor.Direction.FORWARD);
        rightShooter.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        particleCollector.setPower(0);
        leftShooter.setPower(0);
        rightShooter.setPower(0);

        // Set all motors to run with or without encoders.
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        particleCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // No Encoder
        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // coast to a stop
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        /**
         *  Define and set start position on Servos
         */
        BeaconArmsServo = hwMap.servo.get("beaconArms");
        BeaconArmsServo.setPosition(0.5);

        /**
         *  Define and calibrate the Adafruit IMU (Inertial Motion Unit) sensor
         */
        /*gyroSensor = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");
        gyroSensor.calibrate();
        while(gyroSensor.isCalibrating()){
            // Wait for calibration to finish
        }
        gyroSensor.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARDINAL);*/

        /**
         *  Define and setup Color sensors
         */
        leftBeaconColorSensor = hwMap.colorSensor.get("leftBeacon");
        rightBeaconColorSensor = hwMap.colorSensor.get("rightBeacon");
        leftUnderColorSensor = hwMap.colorSensor.get("leftUnder");
        rightUnderColorSensor = hwMap.colorSensor.get("rightUnder");
    }

    public void setShooterPower (double power){
        leftShooter.setPower(power);
        rightShooter.setPower(power);
    }

    public void setDrivePower (double leftPower, double rightPower){
        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}