package org.firstinspires.ftc.teamcode.opmode.teleop;

public class HeadingPID {
    private double kP, kI, kD;
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;

    public HeadingPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double targetHeading, double currentHeading) {
        // Normalize heading error to [-180, 180] for easier computation
        double error = normalizeHeading(targetHeading - currentHeading);

        // Get the current time and calculate delta time
        double currentTime = System.nanoTime() / 1e9; // Convert to seconds
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        if (deltaTime > 0) {
            // Calculate integral and derivative terms
            integralSum += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;

            // Update lastError for the next loop
            lastError = error;

            // Calculate and return the PID correction
            return kP * error + kI * integralSum + kD * derivative;
        }

        // If no time has passed, return 0 correction
        return 0.0;
    }

    private double normalizeHeading(double heading) {
        // Normalize to [-180, 180]
        while (heading > 180) heading -= 360;
        while (heading < -180) heading += 360;
        return heading;
    }

    public void reset() {
        // Resets the PID controller
        integralSum = 0.0;
        lastError = 0.0;
        lastTime = System.nanoTime() / 1e9;
    }
}
