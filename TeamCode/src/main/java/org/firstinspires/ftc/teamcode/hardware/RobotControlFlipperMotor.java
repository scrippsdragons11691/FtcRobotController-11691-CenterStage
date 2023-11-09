package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardwareMap;

public class RobotControlFlipperMotor {
    static final String TAG = "RobotControlFlipperMotor";
    RobotHardwareMap robotHardwareMap;
    LinearOpMode opMode;
    private Telemetry telemetry;
    private boolean dashLogging = true;

    DcMotorEx flipperMotor;
    private boolean flipperInitialized = false;
    ControlModes mode = ControlModes.MANUAL;
    FlipperMotorPositions flipperTargetPosition = FlipperMotorPositions.UNKNOWN;
    FlipperMotorPositions flipperCurrentPosition = FlipperMotorPositions.UNKNOWN;

    public RobotControlFlipperMotor(RobotHardwareMap robotHardwareMap, LinearOpMode opMode){
        this.opMode = opMode;
        this.robotHardwareMap = robotHardwareMap;
        initialize();
    }

    public void initialize(){
        try {
            flipperMotor = robotHardwareMap.baseHMap.get(DcMotorEx.class, "CR");
            flipperMotor.setPower(0);
            flipperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flipperMotor.setDirection(DcMotorEx.Direction.REVERSE);
            flipperMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            flipperMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            flipperInitialized = true;
            opMode.telemetry.addData("flipperMotor", "initialized");
        } catch (IllegalArgumentException iae){
            opMode.telemetry.addData("flipperMotor", iae.getMessage());
        }

        if (dashLogging) {
            telemetry = opMode.telemetry;

            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        }
    }

    public void moveFlipperPower(double power ){
        mode = ControlModes.MANUAL;
        flipperCurrentPosition = FlipperMotorPositions.UNKNOWN;

        //only try moving the arm if initialized
        if (flipperInitialized) {
            double flipperPosition = flipperMotor.getCurrentPosition();
            double clawPowerReducer = 0.3;
            //since we're in manual mode, run without encoder
            flipperMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            if (power > 0
                    && flipperPosition < FlipperMotorPositions.MAX_REVERSE.getEncodedPos()) {
                //telemetry.addData("flipperPower1:", power + " pos " + flipperPosition);
                flipperMotor.setPower(power * clawPowerReducer);
            } else if (power < 0
                    && flipperPosition > FlipperMotorPositions.MAX_FORWARD.getEncodedPos()){
                //telemetry.addData("flipperPower2:", power  + " pos " + flipperPosition);
                flipperMotor.setPower(power * clawPowerReducer);
            } else {
                //telemetry.addData("flipperPowerStop:", power  + " pos " + flipperPosition);
                stopFlipper();
            }
        }
    }

    public void moveFlipperEncoded(FlipperMotorPositions targetPosition){
        mode = ControlModes.AUTO;
        flipperTargetPosition = targetPosition;
        if (flipperInitialized){
            //since we're in auto mode, run with encoder and run to position
            telemetry.addData("flipperAuto:", targetPosition.getPosition());
            flipperMotor.setTargetPosition(targetPosition.getEncodedPos());
            flipperMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            flipperMotor.setPower(0.75);
        }
    }

    public void stopFlipper(){
        if (flipperInitialized){
            //if auto mode, don't set power to 0
            if (mode != ControlModes.AUTO) {
                flipperMotor.setPower(0);
            }
        }
    }

    public double getCurrentPosition(){
        if (flipperInitialized){
            return flipperMotor.getCurrentPosition();
        }
        return 0;
    }

    public void addFlipperTelemetry(){
        if (flipperInitialized){
            telemetry.addData("flipperEncoder:", flipperMotor.getCurrentPosition());
        } else {
            telemetry.addData("flipperEncoder:", "Uninitialized!");
        }
    }
}
