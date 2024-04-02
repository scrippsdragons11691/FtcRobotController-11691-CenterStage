package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.ArmPositions;
import org.firstinspires.ftc.teamcode.hardware.FlipperMotorPositions;
import org.firstinspires.ftc.teamcode.hardware.FlipperPotentiometerPositions;
import org.firstinspires.ftc.teamcode.hardware.GripperPositions;
import org.firstinspires.ftc.teamcode.hardware.Light;
import org.firstinspires.ftc.teamcode.hardware.LightMode;
import org.firstinspires.ftc.teamcode.hardware.PokerPositions;

@Autonomous(name = "Auton Blue Far From Backstage", group = "Autons")
public class AutonBlueFar extends AutonBase {

    private int     parkingPosition= -1;
    private int     lastParkingPosition = -1;

    @Override
    public void runOpMode(){
        boolean left = false;
        boolean middle = false;
        boolean right = true;

        initialize();

        //clawServo1.moveToPosition(GripperPositions.GRIPPER1_CLOSED);
        //clawServo2.moveToPosition(GripperPositions.GRIPPER2_CLOSED);

        while (opModeInInit()) {
            //check that the camera is open and working
            if (robotCameraHandler.frontCameraIsFaulty.get() && robotCameraHandler.enableFrontCamera)
                telemetry.addData("Status", "Front camera fault...Please restart");
            else
                telemetry.addData("Status", "Front Camera initialized");

            try {
                parkingPosition = robotCameraHandler.spikeLocationDetectionPipeline.getSpikeLocation().get();
                telemetry.addData("Parking Position", parkingPosition);
            } catch (NullPointerException npe) {
                telemetry.addData("cameras", npe.getMessage());
            }
            telemetry.update();

            if (lastParkingPosition != parkingPosition) {
                if (parkingPosition == 1) {
                    lights.switchLight(Light.LED1, LightMode.RED);
                } else if (parkingPosition == 2) {
                    lights.switchLight(Light.LED1, LightMode.YELLOW);
                } else if (parkingPosition == 3) {
                    lights.switchLight(Light.LED1, LightMode.GREEN);
                } else if (parkingPosition == 0) {
                    lights.switchLight(Light.LED1, LightMode.OFF);
                }
                lastParkingPosition = parkingPosition;
            }
        }

        waitForStart();

        //Set the arm position up to not drag
        armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_ZERO);
        sleep(2000);
        telemetry.addData("Arm Encoder: ", String.valueOf(armMotor.getArmEncodedPosition()));

        //Left
        if(parkingPosition == 1)
        {
            imuDrive(autonFast, 29.5, 0);
            imuTurn(autonSlow, -90);
            imuDrive(autonSlow, 7, 0);
            imuDrive(autonSlow, -4, 0);

            deliverSpikePixel();

            //return to home
            imuDrive(autonSlow,-5,0);
            imuTurn(autonSlow,90);
            imuDrive(autonSlow,-29.5,0);
            encoderStrafe(autonSlow, -3.5, 5);

        }
        //Middle
        else if(parkingPosition == 2){
            imuDrive(autonSlow, 34, 0);
            sleep(750);
            imuDrive(autonSlow, -3.5, 0);

            deliverSpikePixel();

            imuDrive(autonSlow,-29,0);
        }
        //Right
        else if(parkingPosition == 3){
            imuDrive(autonSlow, 29.5, 0);
            imuTurn(autonSlow, 90);
            imuDrive(autonSlow, 7, 0);
            imuDrive(autonSlow, -2.75, 0);

            deliverSpikePixel();

            imuDrive(autonSlow, -4.75,0 );
            imuTurn(autonSlow, -90);
            imuDrive(autonSlow,-28, 0 );
            encoderStrafe(autonSlow, 2.5, 5);

        }

        //Drive to backdrop
        imuDrive(autonFast,12,0);
        encoderStrafe(autonSlow, 25, 5);
        imuDrive(autonFast, 38, 0);
        imuTurn(autonFast,90);
        imuDrive(autonFast,-96,0);

        //Deliver pixel based on parkingposition
        if(parkingPosition == 3) {
            encoderStrafe(autonSlow, 16, 5);
        }
        else if (parkingPosition == 2)
        {
            encoderStrafe(autonSlow, 23.5, 5);
        }
        else if (parkingPosition == 1)
        {
            encoderStrafe(autonSlow, 35,5);
        }

        //Deliver pixel to backdrop
        imuDrive(autonSlow, -4, 0);

        deliverBackdropPixel();

        //set arm down in the end
        armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_MIN);
        sleep(1000);
    }
}