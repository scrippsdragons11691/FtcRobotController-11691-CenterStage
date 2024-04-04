package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auton Test", group = "Autons")
public class AutonTest extends AutonBase {

    @Override
    public void runOpMode(){

        initialize();
        waitForStart();

        deliverBackdropPixel();
        sleep(5000);
    }
}
