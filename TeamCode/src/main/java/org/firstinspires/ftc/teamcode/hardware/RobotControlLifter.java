package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class RobotControlLifter {
    static final String TAG = "RobotControlLifter";
    RobotHardwareMap robotHardwareMap;
    LinearOpMode opMode;
    private Telemetry telemetry;
    private boolean dashLogging = true;

    DcMotorEx lifterMotor;
    private boolean lifterInitialized = false;
    ControlModes mode = ControlModes.MANUAL;
    //ArmPositions armTargetPosition = ArmPositions.UNKNOWN;
    //ArmPositions armCurrentPosition = ArmPositions.UNKNOWN;


    public RobotControlLifter(RobotHardwareMap robotHardwareMap, LinearOpMode opMode){
        this.opMode = opMode;
        this.robotHardwareMap = robotHardwareMap;
        initialize();
    }

    public void initialize(){
        try {
            lifterMotor = robotHardwareMap.baseHMap.get(DcMotorEx.class, "Lifter");
            lifterMotor.setPower(0);
            lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lifterMotor.setDirection(DcMotorEx.Direction.REVERSE);
            lifterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lifterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            lifterInitialized = true;
            opMode.telemetry.addData("lifter", "initialized");
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("lifter", iae.getMessage());
        }

        if (dashLogging) {
            telemetry = opMode.telemetry;
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }
    }

    public void moveLifterPower(double power ){
        mode = ControlModes.MANUAL;
        //lifterCurrentPosition = ArmPositions.UNKNOWN;

        //only try moving the arm if initilized
        if (lifterInitialized) {
            lifterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double currentPosition = lifterMotor.getCurrentPosition();
            lifterMotor.setPower(power);
            telemetry.addData("Lifter Position",currentPosition);
            telemetry.addData("Lifter Power",power);
        }

    }

    public void stopLifterWithHold(){
        if (lifterInitialized) {

            if (mode == ControlModes.MANUAL) {
                //hold code
                double currentPosition = lifterMotor.getCurrentPosition();
                double holdPower = 0;
                lifterMotor.setPower(holdPower);
            }
        }
    }

    public void stopLifter(){
        if (lifterInitialized){
            lifterMotor.setPower(0);
        }
    }

    public void addLifterTelemetry(){
        if (lifterInitialized){
            telemetry.addData("liftEncoder:", lifterMotor.getCurrentPosition());
        } else {
            telemetry.addData("liftEncoder:", "Uninitialized!");
        }
    }


}
