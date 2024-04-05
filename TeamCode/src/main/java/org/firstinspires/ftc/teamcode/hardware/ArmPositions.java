package org.firstinspires.ftc.teamcode.hardware;

public enum ArmPositions {
    UNKNOWN("UNKNOWN", -1, 0, 0),
    FRONT_ARC_MIN("FRONT_ARC_MIN", 10, 0, 0),
    FRONT_ARC_STRAIGHT("FRONT_ARC_STRAIGHT", 10, 0, 0),
    //ZERO is the drive position  70 as a heigh makes sure we move the prop out of the way during auton
    FRONT_ARC_ZERO("FRONT_ARC_ZERO", 110, 0, 0),
    FIVE_STACK("FIVE_STACK",152,0,0),
    FRONT_ARC_NINETY("FRONT_ARC_NINETY", 443, 0, 0),
    // Value/place value of the arm going up
    FRONT_ARC_TOP("FRONT_ARC_TOP", 1270, 0, 0),
    FRONT_ARC_BOTTOM("FRONT_ARC_BOTTOM", 1270, 0, 0),
    //Apply full power greater than this distance
    RETURN_ARC_TOP("RETURN_ARC_BOTTOM", /*1100*/350, 0, 0),
    BACK_ARC_TOP("BACK_ARC_TOP", 1270, 0, 0),
    CLAW2_HOVER("CLAW2_HOVER", 250, 0, 0),
    CLAW2_PICKUP("CLAW2_PICKUP", 240, 0, 0),
    BACK_ARC_MAX("BACK_ARC_MAX", 1800, 0, 0),
    BACK_ARC_MID("BACK_ARC_MID",800,0,0),
    BACK_ARC_DELIVER("BACK_ARC_DELIVER", 1600, 0, 0),
    ;

    private final String position;
    private final int encodedPos;
    private final double moveToPositionPower;
    private final double moveToPositionVelocity;

    ArmPositions(String position, int encodedPos, double moveToPositionPower, double moveToPositionVelocity){
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
