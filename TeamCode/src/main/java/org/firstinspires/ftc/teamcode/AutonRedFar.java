package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.ArmPositions;
import org.firstinspires.ftc.teamcode.hardware.FlipperMotorPositions;
import org.firstinspires.ftc.teamcode.hardware.GripperPositions;
import org.firstinspires.ftc.teamcode.hardware.Light;
import org.firstinspires.ftc.teamcode.hardware.LightMode;

@Autonomous(name = "Auton Red Far From Backstage", group = "Autons")
public class AutonRedFar extends AutonBase {

    private int     parkingPosition= -1;
    private int     lastParkingPosition = -1;

    @Override
    public void runOpMode(){
        boolean left = false;
        boolean middle = false;
        boolean right = true;

        initialize();

        clawServo1.moveToPosition(GripperPositions.GRIPPER1_CLOSED);
        clawServo2.moveToPosition(GripperPositions.GRIPPER2_CLOSED);

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

        //Left
        if(parkingPosition == 1){
            imuDrive(autonMedium, 21, 0);
            encoderStrafe(autonSlow,-13,5);

            //move arm down to deliver
            deliverSpikePixel();

            //return to home
            imuDrive(autonFast,-21,0);
            encoderStrafe(autonMedium,13,5);

        }
        //Middle
        else if(parkingPosition == 2){
            imuDrive(autonMedium, 34, 0);
            imuDrive(autonSlow, -3, 0);

            //move arm down to deliver
            deliverSpikePixel();

            //drive to return home
            imuDrive(autonFast,-30,0);
        }
        //Right
        else if(parkingPosition == 3) {
            imuDrive(autonMedium, 29.5, 0);
            imuTurn(autonSlow, 90);
            imuDrive(autonSlow, 7, 0);
            imuDrive(autonSlow, -4.5, 0);

            //move arm down to deliver
            deliverSpikePixel();

            //Move back home
            imuDrive(autonMedium,-5.5,0);
            imuTurn(autonMedium,-90);
            imuDrive(autonFast,-28.5,0);
            encoderStrafe(autonMedium,4,5);
        }
        //drive to backboard
        imuDrive(autonFast,12,0);
        encoderStrafe(autonMedium, -25, 5);
        imuDrive(autonFast, 38, 0);
        imuTurn(autonFast,-90);
        imuDrive(autonFast,-103,0);

        //Deliver pixel based on parkingposition
        if(parkingPosition == 1)
        {
            encoderStrafe(autonMedium, -19, 5);
        }
        else if (parkingPosition == 2)
        {
            encoderStrafe(autonMedium, -24.5, 5);
        }
        else if (parkingPosition == 3)
        {
            encoderStrafe(autonMedium, -38,5);
        }

        deliverBackdropPixel();

        //set arm down in the end
        armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_MIN);
    }
}
