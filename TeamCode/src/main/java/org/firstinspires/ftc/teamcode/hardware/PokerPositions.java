package org.firstinspires.ftc.teamcode.hardware;

public enum PokerPositions {
    UNKNOWN("UNKNOWN", 0),
    POKER_FULLOUT("POKER_FULLOUT", 0.65),
    POKER_2PIX("POKER_2PIX", 0.5),
    POKER_1PIX("POKER_1PIX", 0.3),
    POKER_FULLIN("POKER_FULLIN",0.0);

    private final String position;
    private final double servoPos;

    PokerPositions(String position, double servoPos){
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
