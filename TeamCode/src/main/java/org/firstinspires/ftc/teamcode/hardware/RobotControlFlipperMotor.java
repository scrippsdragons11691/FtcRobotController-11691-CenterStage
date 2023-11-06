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
            double clawPowerReducer = 0.5;

            if (power > 0 & flipperPosition < FlipperMotorPositions.MAX_REVERSE.getEncodedPos()) {
                flipperMotor.setPower(power * clawPowerReducer);
            } else if (power < 0 & flipperPosition > FlipperMotorPositions.MAX_FORWARD.getEncodedPos()){
                flipperMotor.setPower(power * clawPowerReducer);
            } else {
                stopFlipper();
            }

            flipperMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            flipperMotor.setPower(power);
        }
    }

    public void stopFlipper(){
        if (flipperInitialized){
            flipperMotor.setPower(0);
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
