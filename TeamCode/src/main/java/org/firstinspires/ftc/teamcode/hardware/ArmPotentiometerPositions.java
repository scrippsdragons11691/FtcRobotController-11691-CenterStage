package org.firstinspires.ftc.teamcode.hardware;

public enum ArmPotentiometerPositions {

    MIN_VOLTAGE("MIN_VOLTAGE", 0),
    PICK_UP("PICK_UP",0.02),
    DRIVE("DRIVE",0.125),
    FIVE_STACK("FIVE_STACK",0.20),
    FIVE_STACK_PICKUP("FIVE_STACK_PICKUP",0.085),
    DELIVER("DELIVER",1.50),
    MAX_VOLTAGE("MAX_VOLTAGE", 4);

    private final String position;
    private final double voltagePos;

    ArmPotentiometerPositions(String position, double voltagePos)
    {
        this.position = position;
        this.voltagePos = voltagePos;
    }

    public String getPosition(){
        return position;
    }

    public double getVoltagePos(){
        return voltagePos;
    }
}
