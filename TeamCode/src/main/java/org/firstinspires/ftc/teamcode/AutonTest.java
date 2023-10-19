package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auton Test", group="Autons")
public class AutonTest extends BaseAuton{

    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        while (opModeIsActive()) {
            driveForward(10, 0.5, 1, false, true, false, false);
        }
    }
}

