package org.firstinspires.ftc.teamcode;

public class ShooterAlgorithm {

    // --- Constants you need to configure ---

    // TODO: Measure the height of your shooter from the floor in centimeters.
    private static final double SHOOTER_HEIGHT_CM = 30.0; // Placeholder value

    // TODO: Characterize your shooter to find the relationship between motor power and exit velocity.
    // For this example, we'll assume a simple linear relationship where motor power scales directly
    // with the velocity of the wheel surface.
    private static final double MAX_VELOCITY_CM_PER_S = 500; // Placeholder: velocity at 1.0 power

    // --- Target information ---
    private static final double BUCKET_HEIGHT_CM = 98.45;
    private static final double BUCKET_FRONT_WIDTH_CM = 67.30;
    private static final double BUCKET_DEPTH_CM = 46.45;

    // --- Physics constants ---
    private static final double GRAVITY_CM_PER_S2 = 981; // 9.81 m/s^2 converted to cm/s^2

    /**
     * Calculates the required angle and motor powers to shoot a pickleball into the bucket.
     *
     * @param distanceToTargetCm The horizontal distance to the front of the bucket in centimeters.
     * @param spinRatio The ratio of bottom motor power to top motor power (e.g., 0.7 for topspin).
     * @return A ShotParameters object with the calculated angle and motor powers, or null if the shot is not possible.
     */
    public static ShotParameters calculateShot(double distanceToTargetCm, double spinRatio) {
        // For simplicity, we'll aim for the center of the bucket.
        double targetX = distanceToTargetCm + BUCKET_DEPTH_CM / 2.0;
        double targetY = BUCKET_HEIGHT_CM;

        // The physics calculations here do not account for the Magnus effect (the lift or dip caused by spin).
        // This is a simplification, but it provides a good starting point. The primary effect of spin
        // is controlled by the spinRatio, which you can tune based on experimental results.

        // Let's iterate through possible angles and see if a solution exists.
        for (double angleDegrees = 0; angleDegrees < 90; angleDegrees += 0.5) {
            double angleRadians = Math.toRadians(angleDegrees);

            // Rearrange the trajectory equations to solve for the required initial velocity (v0).
            // v0^2 = (g * x^2) / (2 * cos^2(theta) * (y0 + x * tan(theta) - y))
            double term1 = 2 * Math.pow(Math.cos(angleRadians), 2);
            double term2 = SHOOTER_HEIGHT_CM + targetX * Math.tan(angleRadians) - targetY;
            if (term1 * term2 <= 0) { // Avoid division by zero or taking the square root of a negative number
                continue;
            }

            double v0_squared = (GRAVITY_CM_PER_S2 * Math.pow(targetX, 2)) / (term1 * term2);

            if (v0_squared > 0) {
                double v0 = Math.sqrt(v0_squared);

                // The linear velocity of the ball (v0) is the average of the top and bottom wheel surface velocities.
                // v0 = (v_top + v_bottom) / 2
                // We also know v_bottom = v_top * spinRatio
                // So, v_top = 2 * v0 / (1 + spinRatio)
                double topWheelVelocity = (2 * v0) / (1 + spinRatio);

                // Check if the required top wheel velocity is achievable.
                if (topWheelVelocity <= MAX_VELOCITY_CM_PER_S) {
                    // We found a valid solution!
                    // Convert the required wheel velocities back to motor powers.
                    double topMotorPower = topWheelVelocity / MAX_VELOCITY_CM_PER_S;
                    double bottomMotorPower = topMotorPower * spinRatio;

                    return new ShotParameters(angleDegrees, topMotorPower, bottomMotorPower);
                }
            }
        }

        // If we get here, no solution was found for the given distance.
        return null;
    }
}
