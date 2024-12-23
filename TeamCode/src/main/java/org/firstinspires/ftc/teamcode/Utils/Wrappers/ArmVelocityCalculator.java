package org.firstinspires.ftc.teamcode.Utils.Wrappers;

public class ArmVelocityCalculator {
    private double previousAngle = 0.0;
    private long previousTime = System.currentTimeMillis();

    // Method to calculate the velocity of the arm
    public double calculateVelocity(double currentAngle) {
        // Get the current time
        long currentTime = System.currentTimeMillis();

        // Calculate the time difference in seconds
        double deltaTime = (currentTime - previousTime) / 1000.0;
        if (deltaTime <= 0) {
            deltaTime = 1e-6; // Avoid division by zero
        }

        // Calculate the change in angle
        double deltaAngle = currentAngle - previousAngle;

        // Calculate the velocity (change in angle over time)
        double velocity = deltaAngle / deltaTime;

        // Update previous values for the next calculation
        previousAngle = currentAngle;
        previousTime = currentTime;

        return velocity;
    }
}