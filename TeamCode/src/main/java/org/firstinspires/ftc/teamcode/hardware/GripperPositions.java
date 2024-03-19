package org.firstinspires.ftc.teamcode.hardware;

public enum GripperPositions {

    UNKNOWN("UNKNOWN", 0),
    GRIPPER1_OPEN("GRIPPER1_OPEN", 0.3),
    GRIPPER1_CLOSED("GRIPPER1_CLOSED", 0.35),
    GRIPPER2_OPEN("GRIPPER2_OPEN", 0.3),
    GRIPPER2_CLOSED("GRIPPER2_CLOSED", 0.19),
    DRONE_READY("DRONE_READY",0.5),
    DRONE_LAUNCH("DRONE_LAUNCH", 0.1);


    private final String position;
    private final double servoPos;

    GripperPositions(String position, double servoPos){
        this.position = position;
        this.servoPos = servoPos;
    }

    public String getPosition(){
        return position;
    }

    public double getServoPos(){
        return servoPos;
    }
}
