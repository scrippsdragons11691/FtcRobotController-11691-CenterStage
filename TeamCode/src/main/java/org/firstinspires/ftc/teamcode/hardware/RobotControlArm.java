package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class RobotControlArm {

    static final String TAG = "RobotControlArm";
    RobotHardwareMap robotHardwareMap;
    LinearOpMode opMode;
    private Telemetry telemetry;
    private boolean dashLogging = true;

    DcMotorEx armMotor;
    private boolean armInitialized = false;
    ControlModes mode = ControlModes.MANUAL;
    ArmPositions armTargetPosition = ArmPositions.UNKNOWN;
    ArmPositions armCurrentPosition = ArmPositions.UNKNOWN;
    RobotControlArmPotentiometer robotControlArmPotentiometer;

    public RobotControlArm(RobotHardwareMap robotHardwareMap, LinearOpMode opMode){
        this.opMode = opMode;
        this.robotHardwareMap = robotHardwareMap;
        initialize();
    }

    public void initialize(){
        try {
            armMotor = robotHardwareMap.baseHMap.get(DcMotorEx.class, "ARM");
            robotControlArmPotentiometer = new RobotControlArmPotentiometer(robotHardwareMap, opMode, "armpot");
            armMotor.setPower(0);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setDirection(DcMotorEx.Direction.REVERSE);
            armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            armInitialized = true;
            opMode.telemetry.addData("arm", "initialized");
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("arm", iae.getMessage());
        }

        if (dashLogging) {
            telemetry = opMode.telemetry;
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }
    }

    public void moveArmPower(double power ){
        mode = ControlModes.MANUAL;
        armCurrentPosition = ArmPositions.UNKNOWN;

        //only try moving the arm if initilized
        if (armInitialized) {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double currentPosition = armMotor.getCurrentPosition();

            double upPowerFactor = 0.8;
            double floatPower = 0.15;

            //if moving up and under apex, then do positive power to move up

            if (power > 0
                    && currentPosition <= ArmPositions.FRONT_ARC_TOP.getEncodedPos()) {
                Log.d(TAG, "moving up  " + currentPosition + " " + power);
                armMotor.setPower(upPowerFactor * power);

                //if moving down and over apex, then do negative power to move back
            } else if (power < 0
                    && currentPosition >= ArmPositions.RETURN_ARC_TOP.getEncodedPos()) {
                Log.d(TAG, "moving up from other side  " + currentPosition + " " + power);
                armMotor.setPower(upPowerFactor * power);

                //if moving down and under apex, then do positive power to slow down
            } else if (power < 0
                    && currentPosition < ArmPositions.FRONT_ARC_BOTTOM.getEncodedPos()) {
                Log.d(TAG, "slowing down  " + currentPosition + " " + power);
                armMotor.setPower(floatPower);

                //if moving up and over apex, then do reverse power
            } else if (power > 0
                    && currentPosition > ArmPositions.BACK_ARC_TOP.getEncodedPos()) {
                Log.d(TAG, "slowing down  " + currentPosition + " " + power);
                armMotor.setPower(floatPower);

            } else {
                Log.d(TAG, "moving no power  " + currentPosition + " " + power);
                armMotor.setPower(power * upPowerFactor);
            }

        }

    }

    public void moveArmEncoded(ArmPositions armTargetPosition){
        if (armInitialized){
            mode = ControlModes.AUTO;
            int currentPosition = armMotor.getCurrentPosition();
            armMotor.setTargetPosition(armTargetPosition.getEncodedPos());
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(ArmMovePowerCalculator.calculatePowerForMove(currentPosition, armTargetPosition.getEncodedPos()));

        }
    }

    public void stopArmWithHold(){
        if (armInitialized) {

            if (mode == ControlModes.MANUAL) {
                //hold code
                double currentPosition = armMotor.getCurrentPosition();
                double holdPower = 0.3;

                //hold position under straight position
                if (currentPosition < ArmPositions.FRONT_ARC_STRAIGHT.getEncodedPos()
                        && currentPosition > ArmPositions.FRONT_ARC_MIN.getEncodedPos()) {
                    armMotor.setPower(holdPower);
                    Log.d(TAG, "holding... " + currentPosition);

                    //hold in back of robot
                } /*else if (currentPosition > ArmPositions.FRONT_ARC_TOP.getEncodedPos()
                        && currentPosition < ArmPositions.BACK_ARC_MAX.getEncodedPos()) {

                    armMotor.setPower(-holdPower);
                    Log.d(TAG, "negative holding..." + currentPosition);

                    //not in a good position for holding
                }*/ else {
                    armMotor.setPower(0);
                    Log.d(TAG, "stopped... " + currentPosition);
                }
            }
        }
    }

    public void stopArmPotWithHold(){
        if (1==1) //(armInitialized)
        {
            //hold code
            double currentPosition = robotControlArmPotentiometer.getCurrentPotPosition();
            double holdPower = 0.3;

            //hold position under straight position
            if (currentPosition > ArmPotentiometerPositions.PICK_UP.getVoltagePos()

                    && currentPosition < ArmPotentiometerPositions.DELIVER.getVoltagePos())
            {
                armMotor.setPower(holdPower);
                Log.d(TAG, "POT holding... " + currentPosition);
            }
            else
            {
                armMotor.setPower(0);
                Log.d(TAG, "stopped... " + currentPosition);
            }
        }
    }


    public void stopArm(){
        if (armInitialized){
            armMotor.setPower(0);
        }
    }
    public void moveArmPowerPot(double power ){
        mode = ControlModes.MANUAL;
        armCurrentPosition = ArmPositions.UNKNOWN;

        //only try moving the arm if initialized
        if (armInitialized) {
            double armPosition = robotControlArmPotentiometer.getCurrentPotPosition();
            double powerReducer = 0.75;

            //since we're in manual mode, run without encoder
            armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            //armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            if (power > 0
                    && armPosition < ArmPotentiometerPositions.MAX_VOLTAGE.getVoltagePos())
            {
                armMotor.setPower(power * powerReducer);
            }
            else if (power < 0
                    && armPosition > ArmPotentiometerPositions.MIN_VOLTAGE.getVoltagePos())
            {
                armMotor.setPower(power * powerReducer);
            }
            else
            {
                //stopArm();
                stopArmPotWithHold();
            }
        }
    }

    public void addArmTelemetry(){
        if (armInitialized){
            telemetry.addData("armEncoder:", armMotor.getCurrentPosition());
            telemetry.addData("armPot:", robotControlArmPotentiometer.getCurrentPotPosition());
        } else {
            telemetry.addData("armEncoder:", "Uninitialized!");
        }
    }

    public double getArmEncodedPosition(){
        return armMotor.getCurrentPosition();
    }
}
