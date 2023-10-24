package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Encoder", group="Autons")
public class AutonEncoderBase extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardwareMap theHardwareMap;
    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.937 ; //3.77953     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public void initialize() {
        theHardwareMap  = new RobotHardwareMap(hardwareMap, this);

        theHardwareMap.initialize();

        theHardwareMap.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        theHardwareMap.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        theHardwareMap.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        theHardwareMap.frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        theHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        theHardwareMap.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        theHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        theHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Starting at",  "%7d :%7d :%7d :%7d",
                theHardwareMap.frontLeftMotor.getCurrentPosition(),
                theHardwareMap.frontRightMotor.getCurrentPosition(),
                theHardwareMap.backLeftMotor.getCurrentPosition(),
                theHardwareMap.backRightMotor.getCurrentPosition()
        );
        telemetry.update();
    }
    public void encoderDriveAndTurn(double speed, double leftInches, double rightInches, double timeoutS) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = theHardwareMap.frontLeftMotor.getCurrentPosition() + (int)(leftInches * RobotGlobalSettings.COUNTS_PER_INCH);
            newBackLeftTarget = theHardwareMap.backLeftMotor.getCurrentPosition() + (int)(leftInches * RobotGlobalSettings.COUNTS_PER_INCH);
            newFrontRightTarget = theHardwareMap.frontRightMotor.getCurrentPosition() + (int)(rightInches * RobotGlobalSettings.COUNTS_PER_INCH);
            newBackRightTarget = theHardwareMap.backRightMotor.getCurrentPosition() + (int)(rightInches * RobotGlobalSettings.COUNTS_PER_INCH);

            theHardwareMap.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            theHardwareMap.backLeftMotor.setTargetPosition(newBackLeftTarget);
            theHardwareMap.frontRightMotor.setTargetPosition(newFrontRightTarget);
            theHardwareMap.backRightMotor.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            theHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            theHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            theHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            theHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            theHardwareMap.frontLeftMotor.setPower(Math.abs(speed));
            theHardwareMap.backLeftMotor.setPower(Math.abs(speed));
            theHardwareMap.frontRightMotor.setPower(Math.abs(speed));
            theHardwareMap.backRightMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (theHardwareMap.frontLeftMotor.isBusy() && theHardwareMap.backLeftMotor.isBusy()
                            && theHardwareMap.frontRightMotor.isBusy() && theHardwareMap.backRightMotor.isBusy())) {
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d",
                        newFrontLeftTarget,  newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                        theHardwareMap.frontLeftMotor.getCurrentPosition(), theHardwareMap.backLeftMotor.getCurrentPosition(),
                        theHardwareMap.frontRightMotor.getCurrentPosition(), theHardwareMap.backRightMotor.getCurrentPosition());
                telemetry.update();
            }

            theHardwareMap.frontLeftMotor.setPower(0);
            theHardwareMap.backLeftMotor.setPower(0);
            theHardwareMap.frontRightMotor.setPower(0);
            theHardwareMap.backRightMotor.setPower(0);

            theHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderStrafe(double speed, double strafeDistance, double timeoutS){
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        if (opModeIsActive()){

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = theHardwareMap.frontLeftMotor.getCurrentPosition() + (int)(strafeDistance * COUNTS_PER_INCH);
            newBackLeftTarget = theHardwareMap.backLeftMotor.getCurrentPosition() + (int)(-strafeDistance * COUNTS_PER_INCH);
            newFrontRightTarget = theHardwareMap.frontRightMotor.getCurrentPosition() + (int)(-strafeDistance * COUNTS_PER_INCH);
            newBackRightTarget = theHardwareMap.backRightMotor.getCurrentPosition() + (int)(strafeDistance * COUNTS_PER_INCH);

            theHardwareMap.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            theHardwareMap.backLeftMotor.setTargetPosition(newBackLeftTarget);
            theHardwareMap.frontRightMotor.setTargetPosition(newFrontRightTarget);
            theHardwareMap.backRightMotor.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            theHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            theHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            theHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            theHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            theHardwareMap.frontLeftMotor.setPower(Math.abs(speed));
            theHardwareMap.backLeftMotor.setPower(Math.abs(speed));
            theHardwareMap.frontRightMotor.setPower(Math.abs(speed));
            theHardwareMap.backRightMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (theHardwareMap.frontLeftMotor.isBusy() && theHardwareMap.backLeftMotor.isBusy()
                            && theHardwareMap.frontRightMotor.isBusy() && theHardwareMap.backRightMotor.isBusy())) {


                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d",
                        newFrontLeftTarget,  newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                        theHardwareMap.frontLeftMotor.getCurrentPosition(), theHardwareMap.backLeftMotor.getCurrentPosition(),
                        theHardwareMap.frontRightMotor.getCurrentPosition(), theHardwareMap.backRightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            theHardwareMap.frontLeftMotor.setPower(0);
            theHardwareMap.backLeftMotor.setPower(0);
            theHardwareMap.frontRightMotor.setPower(0);
            theHardwareMap.backRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            theHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            theHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}