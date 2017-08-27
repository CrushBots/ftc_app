package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.CrushyHardware.SERVO_MAX_POS;
import static org.firstinspires.ftc.teamcode.CrushyHardware.SERVO_MIN_POS;

/**
 * Created by CrushBots for the 2016-2017 FTC season
 */

@TeleOp(name="CommonFunctions", group="Stuff")
@Disabled
public class Auto_CommonFunctions extends LinearOpMode {

    /* Declare OpMode members. */
    protected ElapsedTime runtime = new ElapsedTime();
    CrushyHardware robot = new CrushyHardware();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        //waitForStart();



        //sleep(1000);     // pause for servos to move

    }

    public void DriveInches (int inches) {

        int targetTicks = 0;

        // Diameter of wheel is 4 inches
        // Circumference of wheel is 12.566 inches
        // Ticks per revelotion is 1220

        //double WheelDiameter = 2.88;
        //double WheelCircumfrence = WheelDiameter * Math.PI;
        //distance = distance / WheelCircumfrence;
        //double targetDistance = distance * 1120;

        targetTicks = 97 * inches;


        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for encoders reset
        while(robot.rightBack.getCurrentPosition() != 0 && robot.rightFront.getCurrentPosition() != 0){}
        while(robot.leftBack.getCurrentPosition() != 0 && robot.leftFront.getCurrentPosition() != 0){}

        robot.leftFront.setTargetPosition((int) targetTicks);
        robot.leftBack.setTargetPosition((int) targetTicks);
        robot.rightFront.setTargetPosition((int) targetTicks);
        robot.rightBack.setTargetPosition((int) targetTicks);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.setDrivePower(0.3, 0.3);

        while(robot.leftFront.isBusy() || robot.leftBack.isBusy()){}
        while(robot.rightFront.isBusy() || robot.rightBack.isBusy()){}

        robot.setDrivePower(0, 0);

    }

    public void DriveToLine () {

        robot.setDrivePower(0.1, 0.1);

        robot.setDrivePower(0, 0);

    }

    public void RampUpShooter () {

        double shooterPower = 0;
        while (shooterPower < 0.7){
            shooterPower = Range.clip(shooterPower + 0.01, 0, 0.8);
            robot.setShooterPower(shooterPower);
        }
        sleep(500);
        while (robot.leftShooter.getPower() < 0.7) {}
    }

    public void RampDownShooter () {
        robot.leftShooter.setPower(0.0);
        robot.rightShooter.setPower(0.0);
    }

    public void ShootBalls () {
        RampUpShooter();

        sleep(500);
        robot.particleCollector.setPower(1.0);
        sleep(500);
        robot.particleCollector.setPower(0.0);

        // Step 4: Shoot Ball
        while (robot.leftShooter.getPower() < 0.7) {}
        sleep(500);
        robot.particleCollector.setPower(1.0);
        sleep(500);
        robot.particleCollector.setPower(0.0);

        // Step 5: Ramp down shooter
        RampDownShooter();
    }

    public void ProcessBlueBeacon (){

        sleep(500);

        while (!isBlue(robot.rightBeaconColorSensor)) {
            robot.setDrivePower(0.3, 0.3);
        }
        sleep(500);
        robot.setDrivePower(0, 0);

        // Move arms out
        robot.BeaconArmsServo.setPosition(SERVO_MIN_POS);
        sleep(2000);
        robot.BeaconArmsServo.setPosition(SERVO_MAX_POS);
        sleep(2000);
    }

    public void ProcessRedBeacon (){

        sleep(500);

        while (!isRed(robot.leftBeaconColorSensor)) {
            robot.setDrivePower(0.3, 0.3);
        }
        sleep(1000);
        robot.setDrivePower(0, 0);

        // Move arms out
        robot.BeaconArmsServo.setPosition(SERVO_MIN_POS);
        sleep(2000);
        robot.BeaconArmsServo.setPosition(SERVO_MAX_POS);
        sleep(2000);
    }


    public Boolean isRed(ColorSensor localColorSensor) {

        float hsv[] = {0F,0F,0F};

        localColorSensor.enableLed(false);

        Color.RGBToHSV(localColorSensor.red() * 8, localColorSensor.green() * 8, localColorSensor.blue() * 8, hsv);

        if ((hsv[0] < 30 || hsv[0] > 340) && hsv[1] > .2) {
            return true;
        } else {
            return false;
        }
    }

    public Boolean isWhiteLine(ColorSensor localColorSensor) {

        float hsv[] = {0F,0F,0F};

        localColorSensor.enableLed(true);

        if ((localColorSensor.red() > 180) && (localColorSensor.green() > 180) && (localColorSensor.blue() > 180)){
            return true;
        } else {
            return false;
        }
    }

    public boolean isBlue(ColorSensor localColorSensor) {

        float hsv[] = {0F,0F,0F};

        localColorSensor.enableLed(false);

        Color.RGBToHSV(localColorSensor.red() * 8, localColorSensor.green() * 8, localColorSensor.blue() * 8, hsv);


        if ((hsv[0] > 170 && hsv[0] < 260) && hsv[1] > .2) {
            return true;
        } else {
            return false;
        }
    }

    public void turnRight(int inches){

        int targetTicks = 0;


        targetTicks = 97 * inches;


        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for encoders reset
        int i = 0;
        while(robot.leftBack.getCurrentPosition() != 0 && robot.leftFront.getCurrentPosition() != 0){
            telemetry.addData("reset",i);
            i++;
        }

        robot.leftFront.setTargetPosition((int) targetTicks);
        robot.leftBack.setTargetPosition((int) targetTicks);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setDrivePower(0.3, 0);

        while(robot.leftFront.isBusy() || robot.leftBack.isBusy()){}

        robot.setDrivePower(0, 0);

    }

    public void turnLeft(int inches){

        int targetTicks = 0;


        targetTicks = 97 * inches;


        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for encoders reset
        while(robot.rightBack.getCurrentPosition() != 0 && robot.rightFront.getCurrentPosition() != 0){}

        robot.rightFront.setTargetPosition((int) targetTicks);
        robot.rightBack.setTargetPosition((int) targetTicks);

        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setDrivePower(0.0, 0.3);

        while(robot.rightFront.isBusy() || robot.rightBack.isBusy()){}

        robot.setDrivePower(0, 0);

    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}