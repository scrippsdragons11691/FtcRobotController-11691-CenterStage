package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class BaseAutonIMU {

    public static BNO055IMU imu;
    public static Orientation lastAngles = new Orientation();
    public static double globalAngle;

    public RobotHardwareMap robotHardwareMap;

    // Constructor
    public BaseAutonIMU(RobotHardwareMap hMap)
    {
        robotHardwareMap = hMap;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        //TODO: Determine which IMU should be used (chImu, ehImu, etc.)
        imu = robotHardwareMap.chImu;
        resetAngle();
    }

    public void resetAngle()
    {
        //TODO: Determine correct Axes Order
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public double getAngle()
    {
        //TODO: Determine correct Axes Order
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    protected void SetZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior)
    {
        robotHardwareMap.frontLeftMotor.setZeroPowerBehavior(behavior);
        robotHardwareMap.frontRightMotor.setZeroPowerBehavior(behavior);
        robotHardwareMap.backRightMotor.setZeroPowerBehavior(behavior);
        robotHardwareMap. backLeftMotor.setZeroPowerBehavior(behavior);
    }

    protected void StopAndResetEncoders()
    {
        robotHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardwareMap.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    protected void SetMotorsToRunWithoutEncoders()
    {
        robotHardwareMap.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardwareMap.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardwareMap.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotHardwareMap.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected void setMotorPowerForLinearMove(double LF, double RF, double RR, double LR) {
        robotHardwareMap.frontLeftMotor.setPower(LF);
        robotHardwareMap.frontRightMotor.setPower(RF);
        robotHardwareMap.backRightMotor.setPower(RR);
        robotHardwareMap.backLeftMotor.setPower(LR);
    }

    protected void setMotorPowerForLinearMove(double power) {
        setMotorPowerForLinearMove(power, power, power,power);
    }
}