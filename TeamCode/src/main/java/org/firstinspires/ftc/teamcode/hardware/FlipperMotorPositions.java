package org.firstinspires.ftc.teamcode.hardware;

public enum FlipperMotorPositions {

    UNKNOWN("UNKNOWN", 0, 0, 0),
    CLAW1_DOWN("CLAW1_DOWN", 135, -0.75, 400),
    CLAW2_DOWN("CLAW2_DOWN", 0, -0.75, 400),
    CLAW1_UP("CLAW1_UP", 80, -0.75, 400),
    CLAW2_UP("CLAW2_UP", 145, -0.75, 400),
    MAX_REVERSE("MAX_FORWARD", 200, -0.75, 400),
    MAX_FORWARD("MAX_REVERSE", -50, 0.75, 400);

    private final String position;
    private final int encodedPos;
    private final double moveToPositionPower;
    private final double moveToPositionVelocity;

    FlipperMotorPositions(String position, int encodedPos, double moveToPositionPower, double moveToPositionVelocity){
        this.position = position;
        this.encodedPos = encodedPos;
        this.moveToPositionPower = moveToPositionPower;
        this.moveToPositionVelocity = moveToPositionVelocity;
    }

    public String getPosition(){
        return position;
    }

    public int getEncodedPos(){
        return encodedPos;
    }

    public double getMoveToPositionPower() { return moveToPositionPower;}

    public double getMoveToPositionVelocity() { return moveToPositionVelocity;}
}
