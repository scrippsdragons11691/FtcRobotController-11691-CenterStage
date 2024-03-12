package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class RobotControlPokerServo {
    static final String TAG = "RobotControlPoker";
    RobotHardwareMap robotHardwareMap;
    LinearOpMode opMode;
    private Telemetry telemetry;
    private boolean dashLogging = true;

    String servoLocation = "lower";
    Servo servo;
    private boolean servoInitialized = false;
    ControlModes mode = ControlModes.MANUAL;
    PokerPositions targetPosition = PokerPositions.UNKNOWN;
    PokerPositions currentPosition = PokerPositions.UNKNOWN;

    // Poker Servo
    public RobotControlPokerServo(RobotHardwareMap robotHardwareMap, LinearOpMode opMode, String servoLocation)
    {
        this.opMode = opMode;
        this.robotHardwareMap = robotHardwareMap;
        this.servoLocation = servoLocation;
        initialize();
    }

    public void initialize(){
        try {

            servo = robotHardwareMap.baseHMap.get(Servo.class, servoLocation);
            servoInitialized = true;
            opMode.telemetry.addData(servoLocation, "Initialized");
            servo.setDirection(Servo.Direction.FORWARD);
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData(servoLocation, iae.getMessage());
        }

        if (dashLogging) {
            telemetry = opMode.telemetry;

            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        }
    }

    public void moveToPosition(PokerPositions targetPosition){
        if (currentPosition != targetPosition) {
            Log.d(TAG, "Moving from " + currentPosition + " to " + targetPosition);
            currentPosition = targetPosition;
            if (servoInitialized) {
                servo.setPosition(targetPosition.getServoPos());
            }
        }
    }

    public PokerPositions getCurrentPosition(){
        return currentPosition;
    }
}
