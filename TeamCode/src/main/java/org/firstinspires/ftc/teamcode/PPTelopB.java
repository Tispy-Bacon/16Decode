package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Rotate;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "PPTelop with Controls")
public class PPTelopB extends OpMode {
    // Target Point
    private static final double TARGET_X = 8.0;
    private static final double TARGET_Y = 137.0;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    // PID Controllers
    private final PIDController spinPID = new PIDController(0.02, 0, 0.00008);
    private final PIDController tagPID = new PIDController(0.006, 0, 0.0001);
    private final PIDController turnPID = new PIDController(0.001, 0, 0);
    private Follower follower;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    // Subsystems
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

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize subsystems
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        rotate = new Rotate(hardwareMap);
        CS1_DistanceSensor = hardwareMap.get(DistanceSensor.class, "CS1");

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(15, 70))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 0.6))
                .build();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        handleDriver();
        handleShooterSubsystem();
        updateSpinControl();

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("Yaw (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetryM.debug("Auto-Aim", autoAimEnabled ? "ENABLED" : "OFF");
        telemetryM.debug("Target Distance", Math.hypot(TARGET_X - follower.getPose().getX(), TARGET_Y - follower.getPose().getY()));
        telemetryM.debug("power", shoot_down);
    }

    private void handleDriver() {
        if (!automatedDrive) {
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            double robotYaw = follower.getPose().getHeading();

            if (gamepad1.b) {
                follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
            }

            if (gamepad1.dpad_up) shooter.incrementAngle(.1);
            else if (gamepad1.dpad_down) shooter.incrementAngle(-.1);

            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                // Look at target point
                double targetAngle = Math.atan2(TARGET_Y - follower.getPose().getY(), TARGET_X - follower.getPose().getX());
                double angleError = targetAngle - robotYaw;
                while (angleError > Math.PI) angleError -= 2 * Math.PI;
                while (angleError < -Math.PI) angleError += 2 * Math.PI;
                turn = turnPID.calculate(robotYaw + angleError, robotYaw);
            }

            if (slowMode) {
                forward *= slowModeMultiplier;
                strafe *= slowModeMultiplier;
                turn *= slowModeMultiplier;
            }

            follower.setTeleOpDrive(forward, strafe, turn, false);
        }

        // Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        // Stop automated following
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        // Slow Mode - Using y to avoid conflict with bumper turns
        if (gamepad1.yWasPressed()) {
            slowMode = !slowMode;
        }

        // Adjust slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
            if (slowModeMultiplier > 1.0) slowModeMultiplier = 0.25;
        }
    }

    private void handleShooterSubsystem() {
        // Toggle Auto-Aim with Gamepad 2 Start button
        if (gamepad2.startWasPressed()) {
            autoAimEnabled = !autoAimEnabled;
        }

        if (autoAimEnabled) {
            double distance = Math.hypot(TARGET_X - follower.getPose().getX(), TARGET_Y - follower.getPose().getY());
            shooter.autoAim(distance);
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

        if (gamepad2.dpadLeftWasPressed()) {
            shoot_down -= .1;
            shoot_up -= .1;
        }
        if (gamepad2.dpadRightWasPressed()) {
            shoot_down += .1;
            shoot_up += .1;
        }

        if (gamepad2.leftBumperWasPressed()) {
            targetSpinAngle -= 120;
        }
        if (gamepad2.rightBumperWasPressed()) {
            targetSpinAngle += 120;
        }

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
