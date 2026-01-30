package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A simple PID controller for FTC robots.
 */
public class PIDController {
    private final double Kp, Ki, Kd;
    private double lastError = 0;
    private double integralSum = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public PIDController(double p, double i, double d) {
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;
        timer.reset();
    }

    public double calculate(double target, double current) {
        double error = target - current;
        double dt = timer.seconds();
        
        // Prevent division by zero or extremely high derivative spikes on the first few loops
        if (dt < 0.0001) return 0;

        double derivative = (error - lastError) / dt;
        integralSum += error * dt;

        lastError = error;
        timer.reset();

        return (Kp * error) + (Ki * integralSum) + (Kd * derivative);
    }

    public void reset() {
        lastError = 0;
        integralSum = 0;
        timer.reset();
    }
}
