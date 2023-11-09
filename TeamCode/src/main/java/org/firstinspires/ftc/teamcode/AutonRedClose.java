package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.ArmPositions;
import org.firstinspires.ftc.teamcode.hardware.GripperPositions;
import org.firstinspires.ftc.teamcode.hardware.RobotControlArm;
import org.firstinspires.ftc.teamcode.hardware.RobotControlGripperServos;

@Autonomous(name = "Auton Red Close To Backstage", group = "Autons")
public class AutonRedClose extends AutonBase {

    SpikeLocationDetectionPipeline spikeLocation;
    RobotControlGripperServos clawServo1;
    RobotControlGripperServos clawServo2;
    RobotControlArm armMotor;
    @Override
    public void runOpMode(){
        spikeLocation = new SpikeLocationDetectionPipeline(telemetry);

        clawServo1 = new RobotControlGripperServos(theHardwareMap, this, "ServoClaw1");
        clawServo2 = new RobotControlGripperServos(theHardwareMap, this, "ServoClaw2");
        armMotor = new RobotControlArm(theHardwareMap,this);

        clawServo1.initialize();
        clawServo2.initialize();
        armMotor.initialize();

        boolean left = false;
        boolean middle = false;
        boolean right = true;

        initialize();
        waitForStart();

        //Left
        if(spikeLocation.getSpikeLocation().get() == 1){

        }
        //Middle
        else if(spikeLocation.getSpikeLocation().get() == 2){
            // lift arm before you start driving
            imuDrive(.3, 36, 0);
            sleep(750);
            imuDrive(.15, -5, 0);
            armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_MIN);
            clawServo1.moveToPosition(GripperPositions.GRIPPER1_OPEN);
            clawServo2.moveToPosition(GripperPositions.GRIPPER2_OPEN);
            sleep(250);
            imuDrive(.15, -4, 0);
            imuTurn(.75, -90);
            imuDrive(0.5, -40, 0);
            sleep(5000);
            encoderStrafe(0.5, 28, 5);
            imuDrive(0.5, -13, 0);
        }
        //Right
        else if(spikeLocation.getSpikeLocation().get() == 3){
            encoderStrafe(.4, 5, 0);
            imuDrive(.3, 24, 0);
            sleep(750);
            imuDrive(.3, -8, 0);
            imuTurn(.75, -90);
            imuDrive(0.5, -30, 0);
            //replace delay by placing pixel on backdrop
            sleep(5000);
            encoderStrafe(0.5, -28, 5);
            imuDrive(0.5, 13, 0);
        }
        //Error
        else{
            imuDrive(.3, 36, 0);
            sleep(500);
            imuDrive(.15, -5, 0);
            sleep(2000);
            imuDrive(.15, -4, 0);
            imuTurn(.75, -90);
            imuDrive(0.5, -40, 0);
            //replace delay by placing pixel on backdrop
            sleep(5000);
            encoderStrafe(0.5, 28, 5);
            imuDrive(0.5, -13, 0);
        }
    }
}

