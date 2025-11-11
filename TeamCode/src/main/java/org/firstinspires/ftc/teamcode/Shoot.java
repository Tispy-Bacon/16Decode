package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Shoot {

    private final DcMotor top;
    private final DcMotor bottom;

    public Shoot(DcMotor top, DcMotor bottom) {
        this.top = top;
        this.bottom = bottom;
    }

    public void setPower(double topPower, double bottomPower) {
        top.setPower(topPower);
        bottom.setPower(bottomPower);
    }

    public void stop() {
        top.setPower(0);
        bottom.setPower(0);
    }
}
