package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Spin {

    private final Servo angleL;
    private final Servo angleR;
    private double SAngle = 0.0;

    public Spin(Servo angleL, Servo angleR) {
        this.angleL = angleL;
        this.angleR = angleR;
    }

    public void init() {
        angleL.setDirection(Servo.Direction.REVERSE);
        setAngle(0.0);
    }

    public void setAngle(double angle) {
        this.SAngle = angle;
        angleL.setPosition(SAngle);
        angleR.setPosition(SAngle);
    }

    public void incrementAngle() {
        SAngle += 0.1;
        setAngle(SAngle);
    }

    public void decrementAngle() {
        SAngle -= 0.1;
        setAngle(SAngle);
    }

    public double getAngle(){
        return SAngle;
    }
}
