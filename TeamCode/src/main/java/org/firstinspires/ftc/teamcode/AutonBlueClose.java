package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auton Blue Close To Backstage", group = "Autons")
public class AutonBlueClose extends AutonBase {

    @Override
    public void runOpMode(){

        initialize();
        waitForStart();

        imuDrive(.5, 23, 0);
        sleep(2000);
        imuTurn(.75, -87);
        imuDrive(0.5, 30, 0);
        sleep(5000);
        encoderStrafe(0.5, 26, 5);
        imuDrive(0.5, 13, 0);

    }
}
