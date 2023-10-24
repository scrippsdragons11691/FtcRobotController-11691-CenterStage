package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auton TBD", group = "Autons")
public class AutonOne extends AutonEncoderBase{

    @Override
    public void runOpMode(){
        initialize();
        waitForStart();

        encoderDriveAndTurn(0.5, 48, 48, 5);
        encoderDriveAndTurn(0.5, 32, -32, 5);
        encoderStrafe(0.5, 45, 5);
    }
}
