package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/***
 *
 * robot hardware map
 * Dragon$11691
 */
public class RobotHardwareMap {

    //0 - FL
    //1 - FR
    //2 - RL
    //3 - RR
    private LinearOpMode opMode = null;
    public HardwareMap baseHMap;

    public VoltageSensor controlHubBatteryVoltage;
    //public VoltageSensor expansionHubBatteryVoltage;

    public LynxModule controlHub;
    public LynxModule expansionHub;

    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;
    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;

    public DigitalChannel LED1Green;
    public DigitalChannel LED1Red;
    public DigitalChannel LED2Green;
    public DigitalChannel LED2Red;

    //public ColorSensor elevatorColorSensor;
    //public DigitalChannel leftRedLED;
    //public DigitalChannel leftGreenLED;
    //public DigitalChannel rightRedLED;
    //public DigitalChannel rightGreenLED;
    //public DigitalChannel frontLeftRedLED;
    //public DigitalChannel frontLeftGreenLED;
    //public DigitalChannel frontRightRedLED;
    //public DigitalChannel frontRightGreenLED;

    //public DcMotor leftTowerMotor;
    //public DcMotor rightTowerMotor;
    //public DcMotor intakeMotor;

    public BNO055IMU chImu;
    public BNO055IMU ehImu;
    public BNO055IMU fbImu;

    private final int baseResolution_x = 320;
    private final int baseResolution_y = 240;
    WebcamName frontCamera;

    boolean controlHubBatteryVoltageEnabled = true;
    boolean expansionHubBatteryVoltageEnabled = true;

    public RobotHardwareMap(HardwareMap baseHMap, LinearOpMode opmode) {

        this.opMode = opmode;
        this.baseHMap = baseHMap;
    }


    public void initialize(){

        opMode.telemetry.addData("Status", "detecting...");

        controlHubBatteryVoltage = baseHMap.get(VoltageSensor.class, "Control Hub");
        //expansionHubBatteryVoltage = baseHMap.get(VoltageSensor.class, "Expansion Hub 2");
        controlHub = baseHMap.get(LynxModule.class, "Control Hub");
        //expansionHub = baseHMap.get(LynxModule.class, "Expansion Hub 2");

        //dc motor vs dc motor ex?
        backLeftMotor = baseHMap.get(DcMotorEx.class, "RL");
        backRightMotor = baseHMap.get(DcMotorEx.class, "RR");
        frontLeftMotor = baseHMap.get(DcMotorEx.class, "FL");
        frontRightMotor = baseHMap.get(DcMotorEx.class, "FR");

        try {
            frontCamera = baseHMap.get(WebcamName.class, "Front camera");
            opMode.telemetry.addData("frontCamera", "success ");
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("frontCamera", iae.getMessage());
        }

        //LEDs
        try {
            LED1Green = baseHMap.get(DigitalChannel.class, "LED1green");
            LED1Red = baseHMap.get(DigitalChannel.class, "LED1red");
            LED2Green = baseHMap.get(DigitalChannel.class, "LED2green");
            LED2Red = baseHMap.get(DigitalChannel.class, "LED2red");
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("lights", iae.getMessage());
        }

        // intakeMotor = baseHMap.get(DcMotor.class, "IntakeMotor");

        //try {
        //    leftGreenLED = baseHMap.get(DigitalChannel.class, "leftgreen");
        //} catch (IllegalArgumentException iae){
        //    opMode.telemetry.addData("leftgreen", iae.getMessage());
        //}

        //try {
        //    leftRedLED = baseHMap.get(DigitalChannel.class, "leftred");
        //} catch (IllegalArgumentException iae){
        //    opMode.telemetry.addData("leftred", iae.getMessage());
        //}

        /*
        try {
            rightGreenLED = baseHMap.get(DigitalChannel.class, "rightgreen");
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("rightgreen", iae.getMessage());
        }

        try {
            rightRedLED = baseHMap.get(DigitalChannel.class, "rightred");
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("rightred", iae.getMessage());
        }

        try {
            frontLeftRedLED = baseHMap.get(DigitalChannel.class, "flRed");
            frontLeftGreenLED = baseHMap.get(DigitalChannel.class, "flGreen");
            frontRightRedLED = baseHMap.get(DigitalChannel.class, "frRed");
            frontRightGreenLED = baseHMap.get(DigitalChannel.class, "frGreen");
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("lights", iae.getMessage());
        }
*/
        //try {
        //    coneDistanceSensor = baseHMap.get(DistanceSensor.class, "coneDistance");
        //} catch (IllegalArgumentException iae){
        //    opMode.telemetry.addData("coneDistance", iae.getMessage());
        //}

        /*
        wristServo = hMap.get(Servo.class,"servo1" );
        shirtSensor = hMap.get(ColorSensor.class,"color0" );
        */

        //Initializes the IMU
        /*
        if (chImuEnabled) {
            try {
                chImu = baseHMap.get(BNO055IMU.class, "chImu");
            } catch (IllegalArgumentException iae) {
                opMode.telemetry.addData("chImu", iae.getMessage());
            }
        }

        if (ehImuEnabled) {
            try {
                ehImu = baseHMap.get(BNO055IMU.class, "ehImu");
            } catch (IllegalArgumentException iae) {
                opMode.telemetry.addData("ehImu", iae.getMessage());
            }
        }

        if (fbImuEnabled) {
            try {
                fbImu = baseHMap.get(BNO055IMU.class, "fbImu");
            } catch (IllegalArgumentException iae) {
                opMode.telemetry.addData("fbImu", iae.getMessage());
            }
        }
        */

        opMode.telemetry.addData("Status", "done");
        opMode.telemetry.update();
    }

}
