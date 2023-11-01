package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
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
    public DcMotorEx clawRotator;
    public DcMotorEx armMotor;
    public DigitalChannel LED1Green;
    public DigitalChannel LED1Red;
    public DigitalChannel LED2Green;
    public DigitalChannel LED2Red;

    public Servo servoClaw1;
    public Servo servoClaw2;
    public Servo servoLauncher;

    //public ColorSensor elevatorColorSensor;
    //public DcMotor intakeMotor;

    public IMU chImu;

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
        expansionHub = baseHMap.get(LynxModule.class, "Expansion Hub 2");

        //dc motor vs dc motor ex?
        backLeftMotor = baseHMap.get(DcMotorEx.class, "RL");
        backRightMotor = baseHMap.get(DcMotorEx.class, "RR");
        frontLeftMotor = baseHMap.get(DcMotorEx.class, "FL");
        frontRightMotor = baseHMap.get(DcMotorEx.class, "FR");

        clawRotator = baseHMap.get(DcMotorEx.class, "CR");
        armMotor = baseHMap.get(DcMotorEx.class, "ARM");

        clawRotator.setPower(0);
        clawRotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawRotator.setDirection(DcMotorSimple.Direction.FORWARD);
        clawRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Camera
        try {
            frontCamera = baseHMap.get(WebcamName.class, "Front Camera");
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

        //Servos
        try {
            servoClaw1 = baseHMap.get(Servo.class, "ServoClaw1");
            servoClaw2 = baseHMap.get(Servo.class, "ServoClaw2");
            //servoLauncher = baseHMap.get(Servo.class, "ServoLauncher");
        } catch (IllegalArgumentException iae)
        {
            opMode.telemetry.addData("Servo", iae.getMessage());
        }

        //Initializes the IMU
        chImu = baseHMap.get(IMU.class, "chImu");

        IMU.Parameters myIMUParamaters = new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
        );
        chImu.initialize(myIMUParamaters);

        opMode.telemetry.addData("Status", "done");
        opMode.telemetry.update();
    }

}
