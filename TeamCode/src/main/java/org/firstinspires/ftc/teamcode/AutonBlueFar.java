package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auton Blue Far From Backstage", group = "Autons")
public class AutonBlueFar extends AutonBase {

    @Override
    public void runOpMode(){

        initialize();
        waitForStart();

        //encoderDrive(speed, inches, timeout);
        //encoderStrafe(speed, inches, timeout);
        //imuTurn(speed, degrees);
        imuDrive(0.5, 23.5, 0);
        sleep(2000);
        imuTurn(0.5, -87);
        imuDrive(0.5, 84, 0);
        sleep(4000);
        encoderStrafe(.75, -27, 2000);
        imuDrive(0.5, 12, 0);
    }
}
