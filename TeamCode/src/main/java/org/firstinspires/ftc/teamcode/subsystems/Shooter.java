package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    private final DcMotorEx top, bottom;
    private final Servo angleL, angleR, outtake;

    // Tuning constants for auto-aim (These will need physical calibration)
    private static final double VELOCITY_SLOPE = 50.0; // Increment in velocity per inch
    private static final double VELOCITY_INTERCEPT = 2000.0; // Base velocity
    private static final double ANGLE_BASE = 0.45; // Base servo position
    private static final double ANGLE_COEFF = 0.002; // Change in servo position per inch

    private double currentAngle = ANGLE_BASE; // State to track the current angle position

    public Shooter(HardwareMap hardwareMap) {
        top = hardwareMap.get(DcMotorEx.class, "top");
        bottom = hardwareMap.get(DcMotorEx.class, "bottom");
        
        angleL = hardwareMap.get(Servo.class, "angleL");
        angleR = hardwareMap.get(Servo.class, "angleR");
        outtake = hardwareMap.get(Servo.class, "outtake");

        angleL.setDirection(Servo.Direction.REVERSE);
        
        // Initialize at the base angle
        setAngle(ANGLE_BASE);
    }

    /**
     * Automatically adjusts shooter velocity and angle based on distance to target.
     * @param distanceInches Distance to the basket/target in inches.
     */
    public void autoAim(double distanceInches) {
        if (distanceInches < 0) return; // Invalid distance

        // Linear approximation: v = mx + b
        double targetVelocity = (VELOCITY_SLOPE * distanceInches) + VELOCITY_INTERCEPT;
        
        // Adjust angle slightly based on distance (Trajectory compensation)
        double targetAngle = ANGLE_BASE + (distanceInches * ANGLE_COEFF);

        // Clamp values to safe ranges
        targetVelocity = Math.max(0, Math.min(6000, targetVelocity));
        targetAngle = Math.max(0.3, Math.min(0.7, targetAngle));

        setVelocity(targetVelocity, targetVelocity * 0.9); // Bottom motor slightly slower for backspin
        setAngle(targetAngle);
    }

    public void setVelocity(double topVel, double bottomVel) {
        top.setVelocity(topVel);
        bottom.setVelocity(bottomVel);
    }

    public void setPower(double power) {
        top.setPower(power);
        bottom.setPower(power);
    }

    /**
     * Sets the shooter angle to a specific position and tracks it.
     * @param position The servo position to set (0.0 to 1.0).
     */
    public void setAngle(double position) {
        // Clamp the position to valid servo range [0, 1]
        this.currentAngle = Math.max(0.0, Math.min(1.0, position));
        angleL.setPosition(this.currentAngle);
        angleR.setPosition(this.currentAngle);
    }

    /**
     * Increments the shooter angle by a specified amount.
     * @param delta The amount to change the angle position (can be negative).
     */
    public void incrementAngle(double delta) {
        setAngle(currentAngle + delta);
    }

    /**
     * Returns the current angle tracked by the subsystem.
     */
    public double getCurrentAngle() {
        return currentAngle;
    }

    public void setOuttake(double position) {
        outtake.setPosition(position);
    }
}
