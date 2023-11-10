package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auton Blue Close To Backstage", group = "Autons")
public class AutonBlueClose extends AutonBase {

    @Override
    public void runOpMode(){

        initialize();
        waitForStart();

        //move the arm with the claw to the up position so that it does not drag on the floor position
        //theHardwareMap.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //theHardwareMap.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //theHardwareMap.armMotor.setTargetPosition(90);

        imuDrive(.5, 23, 0);
        sleep(2000);
        imuTurn(.75, -87);
        imuDrive(0.5, 30, 0);
        sleep(5000);
        encoderStrafe(0.5, 26, 5);
        imuDrive(0.5, 13, 0);


    }
}
