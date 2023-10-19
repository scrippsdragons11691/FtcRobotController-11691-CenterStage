package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BaseAuton extends LinearOpMode {

    public enum COMPETITION_SIDE {RED, BLUE};

    RobotHardwareMap robotHardwareMap;
    AutonDrive autonDrive;
    AutonTurn autonTurn;
    ElapsedTime runtime;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    protected void driveForward (double dist, double power, double timeouta, boolean rampDown, boolean brakeAtEnd, boolean incremental, boolean nearest90){
        autonDrive.encoderDriveAuton (dist, power,timeouta, this, rampDown, brakeAtEnd, incremental, nearest90);
    }

    protected void driveBackward (double dist, double power, double timeouta, boolean rampDown, boolean brakeAtEnd, boolean incremental, boolean nearest90){
        autonDrive.encoderDriveAuton (dist * -1, power,timeouta, this, rampDown, brakeAtEnd, incremental, nearest90);
    }

    protected void straff (double dist, double power, double timeoutb){
        autonDrive.Auton_Straff (dist,power,timeoutb, this);
    }

    protected void turn_HighPowerAtEnd (double angle, double powerturn, double deltaPower, double timeoutc){
        autonTurn.AutonTurn_HighPowerAtEnd (angle, powerturn, deltaPower, timeoutc, this);
    }

    protected void initialize(){
        runtime          = new ElapsedTime();
        robotHardwareMap = new RobotHardwareMap(hardwareMap, this);
        autonDrive       = new AutonDrive(robotHardwareMap);
        autonTurn        = new AutonTurn(robotHardwareMap);
    }
}

