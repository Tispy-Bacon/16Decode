package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Rotate;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/**
 * Refactored TeleopBase using Subsystems, External PIDController, and Auto-Aim.
 */
@TeleOp(name = "TeleopBase Refactored")
public class TeleopBase extends LinearOpMode {

    // Subsystems
    private MecanumDrive drive;
    private Vision vision;
    private Shooter shooter;
    private Intake intake;
    private Rotate rotate;
    private DistanceSensor CS1_DistanceSensor;

    // State Variables
    private double shoot_down = 0.7;
    private double shoot_up = 0.6;
    private double SAngle = 0.5;
    private double targetSpinAngle = 0;
    private boolean SToggle = false;
    private boolean autoAimEnabled = false;

    // PID Controllers
    private final PIDController spinPID = new PIDController(0.02, 0, 0.00008);
    private final PIDController tagPID = new PIDController(0.006, 0, 0.0001);
    private final PIDController turnPID = new PIDController(0.001, 0, 0);

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized with Auto-Aim");
        telemetry.update();

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            vision.update();

            handleDriver();
            handleShooter();
            updateSpinControl();

            if (loopTimer.milliseconds() > 50) {
                telemetry.addData("Loop Rate (Hz)", Math.round(1.0 / loopTimer.seconds()));
                telemetry.addData("Yaw (Deg)", Math.toDegrees(drive.getYaw()));
                telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED" : "OFF");
                telemetry.addData("Tag Distance", vision.getDistanceToTag(0));
                telemetry.addData("power", shoot_down );
                telemetry.update();
                loopTimer.reset();
            }
        }
    }

    private void initHardware() {
        drive = new MecanumDrive(hardwareMap);
        vision = new Vision(hardwareMap, "shootCam");
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        rotate = new Rotate(hardwareMap);
        
        CS1_DistanceSensor = hardwareMap.get(DistanceSensor.class, "CS1");
    }

    private void handleDriver() {
        if (gamepad1.b) drive.resetYaw();

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double robotYaw = drive.getYaw();

        if (gamepad1.dpad_up) shooter.incrementAngle(.1);
        else if (gamepad1.dpad_down) shooter.incrementAngle(-.1);
        ;

        if (gamepad1.right_bumper) {
            turn = turnPID.calculate(Math.toRadians(-45), robotYaw);
        } else if (gamepad1.left_bumper) {
            turn = turnPID.calculate(Math.toRadians(45), robotYaw);
        } else if (gamepad1.right_trigger > 0.1) {
            int tagX = vision.getTargetTagX(0);
            if (tagX != -1) {
                turn = -tagPID.calculate(150, tagX);
            }
        }

        drive.driveFieldRelative(forward, strafe, turn, robotYaw);
    }

    private void handleShooter() {
        // Toggle Auto-Aim with Gamepad 2 Start button
        if (gamepad2.startWasPressed()) {
            autoAimEnabled = !autoAimEnabled;
        }

        if (autoAimEnabled) {
            double distance = vision.getDistanceToTag(0); // Assuming tag 0 is the basket
            if (distance != -1) {
                shooter.autoAim(distance);
            }
        } else {
            // Manual shooter control
            if (gamepad2.aWasPressed()) {
                SToggle = !SToggle;
                if (SToggle) {
                    shooter.setVelocity(6000 * shoot_up, 6000 * shoot_down);
                    targetSpinAngle += 60;
                } else {
                    shooter.setPower(0);
                    targetSpinAngle -= 60;
                }
            }
            shooter.setAngle(SAngle);
        }

        if (gamepad2.dpad_up) SAngle += 0.02;
        if (gamepad2.dpad_down) SAngle -= 0.02;

        if (gamepad2.dpadLeftWasPressed()) {shoot_down -= .1; shoot_up -= .1;}
        if (gamepad2.dpadRightWasPressed()) {shoot_down += .1; shoot_up += .1;}

        if (gamepad2.leftBumperWasPressed()) {targetSpinAngle -= 120;}
        if (gamepad2.rightBumperWasPressed()) {targetSpinAngle += 120;}



        if (gamepad2.y) intake.setPower(-1);
        else if (gamepad2.x) intake.setPower(1);
        else intake.setPower(0);

        if (gamepad2.right_trigger > 0.3) {
            shooter.setOuttake(0.3);
            if (gamepad2.b && CS1_DistanceSensor.getDistance(DistanceUnit.CM) > 3.8) {
                targetSpinAngle += 120;
            }
        } else {
            shooter.setOuttake(0);
        }
    }

    private void updateSpinControl() {
        double currentAngle = rotate.getAngle();
        double output = spinPID.calculate(targetSpinAngle, currentAngle);
        rotate.setRotatePower(-output);
    }
}
