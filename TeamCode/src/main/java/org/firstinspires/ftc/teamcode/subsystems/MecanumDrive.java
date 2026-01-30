package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class MecanumDrive {
    private final IMU imu;
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    public MecanumDrive(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    public void driveFieldRelative(double forward, double strafe, double turn, double robotYaw) {
        double rx = strafe * Math.cos(-robotYaw) - forward * Math.sin(-robotYaw);
        double ry = strafe * Math.sin(-robotYaw) + forward * Math.cos(-robotYaw);

        double denominator = Math.max(Math.abs(ry) + Math.abs(rx) + Math.abs(turn), 1.0);
        frontLeft.setPower((ry + rx + turn) / denominator);
        backLeft.setPower((ry - rx + turn) / denominator);
        frontRight.setPower((ry - rx - turn) / denominator);
        backRight.setPower((ry + rx - turn) / denominator);
    }

    public double getYaw() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.RADIANS);
    }

    public void resetYaw() {
        imu.resetYaw();
    }
}
