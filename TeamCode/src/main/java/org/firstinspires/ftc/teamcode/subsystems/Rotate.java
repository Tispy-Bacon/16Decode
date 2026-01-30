package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Rotate {
    private final DcMotorEx uppies;
    private final CRServo rotate;

    public Rotate(HardwareMap hardwareMap) {
        uppies = hardwareMap.get(DcMotorEx.class, "uppies");
        uppies.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uppies.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        rotate = hardwareMap.get(CRServo.class, "rotate");
    }
    public void setRotatePower(double power) {
        rotate.setPower(power);
    }

    public int getPosition() {
        return uppies.getCurrentPosition();
    }
    
    public double getAngle() {
        return getPosition() / (8192.0 / 360.0);
    }
}
