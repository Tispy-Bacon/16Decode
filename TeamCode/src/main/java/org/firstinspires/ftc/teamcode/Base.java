package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Base (Blocks to Java)")
public class Base extends LinearOpMode {

    private IMU imu;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    /**
     * This takes field relative, converts to field relative and drives the robot
     */
    private void drive_relative(float forward_relative, float right_relative, float rotate_relative) {
        double theta;
        double r;

        theta = Math.atan2(forward_relative, right_relative) / Math.PI * 180;
        r = Math.sqrt(forward_relative * forward_relative + right_relative * right_relative);
        theta = AngleUnit.DEGREES.normalize(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        drive(r * Math.sin(theta / 180 * Math.PI), r * Math.cos(theta / 180 * Math.PI), rotate_relative);
    }

    /**
     * This OpMode illustrates how to program your robot to drive field relative. This means
     * that the robot drives the direction you push the joystick regardless of the current orientation of the robot.
     * <p>
     * This OpMode assumes that you have four mechanum wheels each on its own motor named:
     *  front_left_motor, front_right_motor, back_left_motor, back_right_motor
     * <p>
     * and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
     */
    @Override
    public void runOpMode() {
        imu = hardwareMap.get(IMU.class, "imu");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        Servo angleL = hardwareMap.get(Servo.class, "angleL");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        Servo angleR = hardwareMap.get(Servo.class, "angleR");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor top = hardwareMap.get(DcMotor.class, "top");
        DcMotor bottom = hardwareMap.get(DcMotor.class, "bottom");

        Shoot shoot = new Shoot(top, bottom);
        Take take = new Take(intake);
        Spin spin = new Spin(angleL, angleR);

        // Put initialization blocks here.
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        spin.init();
        // Create a RevHubOrientationOnRobot object for use with an IMU in a REV Robotics Control
        // Hub or Expansion Hub, specifying the hub's orientation on the robot via the direction
        // that the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            intake.setDirection(DcMotor.Direction.REVERSE);
            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.addLine("Press Options to reset Yaw");
                telemetry.addLine("The left joystick sets the robot direction");
                telemetry.addLine("Moving the right joystick left and right turns the robot");
                telemetry.addLine("Moving the right joystick left and right turns the robot");
                if (gamepad1.b) {
                    imu.resetYaw();
                }
                if (gamepad1.yWasPressed()) {
                    // TODO: Replace this with a reading from your distance sensor
                    double distanceToTarget = 200; // 2 meters
                    double spinRatio = 0.7; // Topspin
                    ShotParameters shot = ShooterAlgorithm.calculateShot(distanceToTarget, spinRatio);
                    if (shot != null) {
                        spin.setAngle(shot.angle);
                        shoot.setPower(shot.topMotorPower, shot.bottomMotorPower);
                        telemetry.addData("Shot Angle", shot.angle);
                        telemetry.addData("Top Motor", shot.topMotorPower);
                        telemetry.addData("Bottom Motor", shot.bottomMotorPower);
                    } else {
                        telemetry.addLine("Shot not possible at this distance");
                    }
                }
                if (gamepad1.aWasPressed()) {
                    // For manual shooting, you can use a default power with spin
                    shoot.setPower(1.0, 0.7);
                }
                if (gamepad1.xWasPressed()) {
                    take.execute();
                }
                if (gamepad1.dpadUpWasPressed()) {
                    spin.incrementAngle();
                }
                if (gamepad1.dpadDownWasPressed()) {
                    spin.decrementAngle();
                }
                drive_relative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
                telemetry.addData("Angle", spin.getAngle());
                telemetry.update();
            }
        }
    }

    /**
     * Thanks to FTC16072 for sharing this code!
     */
    private void drive(double forward, double right, float rotate2) {
        double maxPower;
        double frontLeftPower;
        double frontRightPower;
        double backRightPower;
        double backLeftPower;
        int maxSpeed;

        maxPower = 1;
        frontLeftPower = forward + right + rotate2;
        frontRightPower = forward - (right + rotate2);
        backRightPower = forward + (right - rotate2);
        backLeftPower = forward - (right - rotate2);
        // This is needed to make sure we don't pass > 1.0 to any wheel.
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        // Change maxSpeed to be less than 1 for outreaches.  Do NOT change to be greater than 1
        maxSpeed = 1;
        frontLeft.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRight.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeft.setPower(maxSpeed * (backLeftPower / maxPower));
        backRight.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
