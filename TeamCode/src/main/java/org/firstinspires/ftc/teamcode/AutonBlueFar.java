package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.ArmPositions;
import org.firstinspires.ftc.teamcode.hardware.FlipperMotorPositions;
import org.firstinspires.ftc.teamcode.hardware.GripperPositions;
import org.firstinspires.ftc.teamcode.hardware.Light;
import org.firstinspires.ftc.teamcode.hardware.LightMode;

@Autonomous(name = "Auton Blue Far From Backstage", group = "Autons")
public class AutonBlueFar extends AutonBase {

    private int     parkingPosition= -1;
    private int     lastParkingPosition = -1;

    private double autonFast = 0.7;
    private double autonSlow = 0.25;

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
        sleep(2000);
        telemetry.addData("Arm Encoder: ", String.valueOf(armMotor.getArmEncodedPosition()));

        //Left
        if(parkingPosition == 1){
            imuDrive(.4, 29.5, 0);
            imuTurn(.3, -90);
            imuDrive(.25, 7, 0);
            imuDrive(.15, -4, 0);

            //move arm down to deliver
            armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_MIN);
            sleep(500);
            clawServo2.moveToPosition(GripperPositions.GRIPPER2_OPEN);
            sleep(1000);
            armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_ZERO);
            sleep(1000);

            //return to home
            imuDrive(.4,-5,0);
            imuTurn(.3,90);
            imuDrive(.4,-29.5,0);
            encoderStrafe(0.25, -3.5, 5);


            //drive to deliver pixel
/*
            imuDrive(0.25, -5.5, 0);
            imuTurn(0.5,180);
            encoderStrafe(0.5, -29, 5);
            imuDrive(0.5, -85, 0);
            encoderStrafe(0.25, 43, 5);

            armMotor.moveArmEncoded(ArmPositions.BACK_ARC_MAX);
            sleep(500);
            flipper.moveFlipperEncoded(FlipperMotorPositions.CLAW2_PLACE);
            sleep(250);
            imuDrive(.15, -5.5, 0);
            sleep(250);
            clawServo1.moveToPosition(GripperPositions.GRIPPER1_OPEN);
            sleep(250);
            armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_ZERO);
            sleep(1000);

 */
        }
        //Middle
        else if(parkingPosition == 2){
            imuDrive(.3, 34, 0);
            sleep(750);
            imuDrive(.15, -3.5, 0);

            //move arm down to deliver
            clawServo2.moveToPosition(GripperPositions.GRIPPER2_OPEN);
            sleep(1000);
            armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_ZERO);
            sleep(1000);

            imuDrive(.3,-29,0);

            //drive to deliver pixel
            /*
            imuDrive(.25, -10, 0);
            encoderStrafe(.5,12,5);
            imuDrive(.5,30,0);
            imuTurn(.75, 90);
            imuDrive(0.5, -94, 0);
            encoderStrafe(0.25, 27, 5);

            armMotor.moveArmEncoded(ArmPositions.BACK_ARC_MAX);
            sleep(500);
            flipper.moveFlipperEncoded(FlipperMotorPositions.CLAW2_PLACE);
            sleep(300);
            imuDrive(.15, -4, 0);
            sleep(250);
            clawServo1.moveToPosition(GripperPositions.GRIPPER1_OPEN);
            sleep(250);
            armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_ZERO);
            sleep(1000);
*/

        }
        //Right
        else if(parkingPosition == 3){
            imuDrive(.4, 29.5, 0);
            imuTurn(.3, 90);
            imuDrive(.25, 7, 0);
            imuDrive(.15, -2.75, 0);

            //move arm down to deliver
            armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_MIN);
            sleep(500);
            clawServo2.moveToPosition(GripperPositions.GRIPPER2_OPEN);
            sleep(500);
            armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_ZERO);
            sleep(500);
            imuDrive(.25, -4.75,0 );
             imuTurn(.3, -90);
             imuDrive(.5,-28, 0 );

            //drive towards back
            /*imuDrive(.5, -3.5, 0);
            encoderStrafe(0.5,-28,5);
            imuDrive(.5,-84,0);
            sleep(250);
            encoderStrafe(0.5, 20, 5);

            armMotor.moveArmEncoded(ArmPositions.BACK_ARC_MAX);
            sleep(500);re
            flipper.moveFlipperEncoded(FlipperMotorPositions.CLAW2_PLACE);
            sleep(300);
            imuDrive(.15, -3.5, 0);
            sleep(250);
            clawServo1.moveToPosition(GripperPositions.GRIPPER1_OPEN);
            sleep(250);
            armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_ZERO);
            sleep(1000);
            */



        }
        //Error unable to find target so slide to backdrop
        else{
            imuDrive(.5, 50, 0);
            imuTurn(0.5,90);
            imuDrive(0.5,-93,0);

            telemetry.addData("Park Position Unknown",parkingPosition);
        }

        //Drive to backdrop
        imuDrive(autonFast,12,0);
        encoderStrafe(autonSlow, 25, 5);
        imuDrive(autonFast, 36, 0);
        imuTurn(autonFast,90);
        imuDrive(autonFast,-96,0);

        //Deliver pixel based on parkingposition
        if(parkingPosition == 3)
            encoderStrafe(autonSlow,16,5);

        else if (parkingPosition ==2) {
            encoderStrafe(autonSlow, 23.5, 5);
        }

        else if (parkingPosition ==1) {
                encoderStrafe(autonSlow, 35,5);



            }
        //Deliver pixel to backdrop
        imuDrive(autonSlow, -4, 0);
        armMotor.moveArmEncoded(ArmPositions.BACK_ARC_MAX);
        sleep(500);
        flipper.moveFlipperEncoded(FlipperMotorPositions.CLAW2_PLACE);
        sleep(300);
        imuDrive(.15, -6.5, 0);
        sleep(250);
        clawServo1.moveToPosition(GripperPositions.GRIPPER1_OPEN);
        sleep(250);
        armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_ZERO);
        sleep(1000);




        //set arm down in the end
        armMotor.moveArmEncoded(ArmPositions.FRONT_ARC_MIN);
    }
}
