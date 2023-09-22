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
    public VoltageSensor expansionHubBatteryVoltage;

    public LynxModule controlHub;
    public LynxModule expansionHub;

    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;
    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;

    public DcMotorEx elevatorMotor;
    public TouchSensor elevatorMotorLowerLimitSwitch;
    public TouchSensor elevatorMotorUpperLimitSwitch;
    public ColorSensor elevatorColorSensor;
    public TouchSensor fourBarLowerLimitSwitch;
    public DcMotorEx fourBarMotor;
    public AnalogInput fourBarPot;
    public DigitalChannel leftRedLED;
    public DigitalChannel leftGreenLED;
    public DigitalChannel rightRedLED;
    public DigitalChannel rightGreenLED;
    public DigitalChannel frontLeftRedLED;
    public DigitalChannel frontLeftGreenLED;
    public DigitalChannel frontRightRedLED;
    public DigitalChannel frontRightGreenLED;

    public DcMotor leftTowerMotor;
    public DcMotor rightTowerMotor;
    public DcMotor intakeMotor;

    public Servo clawServo;
    public Servo wristServo;
    public ColorSensor shirtSensor;
    public DistanceSensor coneDistanceSensor;
    public BNO055IMU chImu;
    public BNO055IMU ehImu;
    public BNO055IMU fbImu;

    private final int baseResolution_x = 320;
    private final int baseResolution_y = 240;
    WebcamName parkingCamera;
    WebcamName frontCamera;

    boolean controlHubBatteryVoltageEnabled = true;
    boolean expansionHubBatteryVoltageEnabled = true;
    boolean backLeftMotorEnabled = true;
    boolean backRightMotorEnabled = true;
    boolean frontLeftMotorEnabled = true;
    boolean frontRightMotorEnabled = true;
    boolean elevatorMotorEnabled = true;
    boolean elevatorMotorLowerLimitSwitchEnabled = true;
    boolean elevatorMotorUpperLimitSwitchEnabled = true;
    boolean elevatorColorSensorEnabled = false;
    boolean fourBarLowerLimitSwitchEnabled = true;
    boolean fourBarMotorEnabled = true;
    boolean fourBarPotEnabled = false;
    boolean leftRedLEDEnabled = true;
    boolean leftGreenLEDEnabled = true;
    boolean rightRedLEDEnabled = true;
    boolean rightGreenLEDEnabled = true;
    boolean frontLeftRedLEDEnabled = true;
    boolean frontLeftGreenLEDEnabled = true;
    boolean frontRightRedLEDEnabled = true;
    boolean frontRightGreenLEDEnabled = true;
    boolean clawServoEnabled = true;
    boolean coneDistanceSensorEnabled = true;
    boolean chImuEnabled = true;
    boolean ehImuEnabled = true;
    boolean fbImuEnabled = true;
    boolean parkingCameraEnabled = true;
    boolean frontCameraEnabled = true;

    public RobotHardwareMap(HardwareMap baseHMap, LinearOpMode opmode) {

        this.opMode = opmode;
        this.baseHMap = baseHMap;
    }


    public void initialize(){

        opMode.telemetry.addData("Status", "detecting...");

        controlHubBatteryVoltage = baseHMap.get(VoltageSensor.class, "Control Hub");
        expansionHubBatteryVoltage = baseHMap.get(VoltageSensor.class, "Expansion Hub 2");
        controlHub = baseHMap.get(LynxModule.class, "Control Hub");
        expansionHub = baseHMap.get(LynxModule.class, "Expansion Hub 2");


        //dc motor vs dc motor ex?
        backLeftMotor = baseHMap.get(DcMotorEx.class, "RL");
        backRightMotor = baseHMap.get(DcMotorEx.class, "RR");
        frontLeftMotor = baseHMap.get(DcMotorEx.class, "FL");
        frontRightMotor = baseHMap.get(DcMotorEx.class, "FR");

        //leftTowerMotor = baseHMap.get(DcMotor.class, "TurntableMotorLeft");
        // rightTowerMotor = baseHMap.get(DcMotor.class, "TurntableMotorRight");
        // intakeMotor = baseHMap.get(DcMotor.class, "IntakeMotor");


        if (elevatorMotorEnabled) {
            try {
                elevatorMotor = baseHMap.get(DcMotorEx.class, "elevatorMotor");
            } catch (IllegalArgumentException iae) {
                opMode.telemetry.addData("elevatorMotor", iae.getMessage());
            }
        }

        if (fourBarMotorEnabled) {
            try {
                fourBarMotor = baseHMap.get(DcMotorEx.class, "fourBarMotor");
            } catch (IllegalArgumentException iae) {
                opMode.telemetry.addData("fourBarMotor", iae.getMessage());
            }
        }

        if (clawServoEnabled) {
            try {
                clawServo = baseHMap.get(Servo.class, "clawServo");
            } catch (IllegalArgumentException iae) {
                opMode.telemetry.addData("clawServo", iae.getMessage());
            }
        }

        if (fourBarLowerLimitSwitchEnabled) {
            try {
                fourBarLowerLimitSwitch = baseHMap.get(TouchSensor.class, "fLimit");
            } catch (IllegalArgumentException iae) {
                opMode.telemetry.addData("fourBarLowerLimitSwitch", iae.getMessage());
            }
        }

        try {
            elevatorMotorLowerLimitSwitch = baseHMap.get(TouchSensor.class, "eLimitDown");
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("elevatorMotorLowerLimitSwitch", iae.getMessage());
        }

        try {
            elevatorMotorUpperLimitSwitch = baseHMap.get(TouchSensor.class, "eLimitUp");
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("elevatorMotorUpperLimitSwitch", iae.getMessage());
        }

        if (fourBarPotEnabled) {
            try {
                fourBarPot = baseHMap.get(AnalogInput.class, "fourBarPot");
            } catch (IllegalArgumentException iae) {
                opMode.telemetry.addData("fourBarPot", iae.getMessage());
            }
        }

        if (elevatorColorSensorEnabled) {
            try {
                elevatorColorSensor = baseHMap.get(ColorSensor.class, "ecolor");
            } catch (IllegalArgumentException iae) {
                opMode.telemetry.addData("elevatorColorSensor", iae.getMessage());
            }
        }

        try {
            leftGreenLED = baseHMap.get(DigitalChannel.class, "leftgreen");
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("leftgreen", iae.getMessage());
        }

        try {
            leftRedLED = baseHMap.get(DigitalChannel.class, "leftred");
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("leftred", iae.getMessage());
        }

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

        try {
            coneDistanceSensor = baseHMap.get(DistanceSensor.class, "coneDistance");
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("coneDistance", iae.getMessage());
        }

        try {
            parkingCamera = baseHMap.get(WebcamName.class, "Parking camera");
            opMode.telemetry.addData("parkingCamera", "success ");
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("parkingCamera", iae.getMessage());
        }

        try {
            frontCamera = baseHMap.get(WebcamName.class, "Front camera");
            opMode.telemetry.addData("frontCamera", "success ");
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("frontCamera", iae.getMessage());
        }

        /*
        wristServo = hMap.get(Servo.class,"servo1" );
        shirtSensor = hMap.get(ColorSensor.class,"color0" );
        */

        //Initializes the IMU
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


        opMode.telemetry.addData("Status", "done");
        opMode.telemetry.update();



    }

}
