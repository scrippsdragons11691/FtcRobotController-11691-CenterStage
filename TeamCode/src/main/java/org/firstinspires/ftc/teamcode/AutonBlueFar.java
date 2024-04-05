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
        telemetry.addData("Arm Encoder: ", String.valueOf(armMotor.getArmEncodedPosition()));

        //Left
        if(parkingPosition == 1)
        {
            imuDrive(autonFast, 29.5, 0);
            imuTurn(autonMedium, -90);
            imuDrive(autonMedium, 7, 0);
            imuDrive(autonMedium, -4, 0);

            deliverSpikePixel();

            //return to home
            imuDrive(autonMedium,-5,0);
            imuTurn(autonMedium,90);
            imuDrive(autonFast,-29.5,0);
            encoderStrafe(autonMedium, -4.5, 5);

        }
        //Middle
        else if(parkingPosition == 2){
            imuDrive(autonMedium, 34, 0);
            imuDrive(autonSlow, -3.5, 0);

            deliverSpikePixel();

            imuDrive(autonMedium,-29,0);
        }
        //Right
        else if(parkingPosition == 3){
            imuDrive(autonMedium, 21, 0);
            encoderStrafe(autonSlow,13,5);

            //move arm down to deliver
            deliverSpikePixel();

            //return to home
            imuDrive(autonFast,-21,0);
            encoderStrafe(autonMedium,-13,5);
        }

        //Drive to backdrop
        imuDrive(autonFast,12,0);
        encoderStrafe(autonMedium, 25, 5);
        imuDrive(autonFast, 37, 0);
        armMotor.moveArmEncoded(ArmPositions.FIVE_STACK);
        imuTurn(autonFast,90);

        //Go for white pixel
        imuDrive(autonSlow,6,0);
        servoPoker.moveToPosition(PokerPositions.POKER_FULLOUT);
        sleep(2000);
        armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_ZERO);
        imuDrive(autonMedium,-6,0);

        imuDrive(autonFast,-100,0);

        //Deliver pixel based on parkingposition
        if(parkingPosition == 3) {
            encoderStrafe(autonMedium, 16, 5);
        }
        else if (parkingPosition == 2)
        {
            encoderStrafe(autonMedium, 23.5, 5);
        }
        else if (parkingPosition == 1)
        {
            encoderStrafe(autonMedium, 33,5);
        }

        //Deliver pixel to backdrop
        imuDrive(autonSlow, -4, 0);

        deliverBackdropPixel();

        //set arm down in the end
        armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_MIN);
        sleep(1000);
    }
}