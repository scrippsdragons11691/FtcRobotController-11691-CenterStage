package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auton Red Close To Backstage", group = "Autons")
public class AutonRedClose extends AutonBase {

    @Override
    public void runOpMode(){

        initialize();
        waitForStart();

        imuDrive(.5, 23, 0);
        sleep(2000);
        imuTurn(.75, 87);
        imuDrive(0.5, 30, 0);
        sleep(5000);
        encoderStrafe(0.5, -30, 5);
        imuDrive(0.5, 13, 0);

    }
}
