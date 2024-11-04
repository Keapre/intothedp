package org.firstinspires.ftc.teamcode.Utils;

public class MotionProfile {
    private double initialPosition;
    private double finalPosition;
    private double maxVelocity;
    private double maxAcceleration;
    private double totalTime;
    private double t1; // Acceleration phase duration
    private double t2; // Constant velocity phase duration
    private double t3; // Deceleration phase duration

    public MotionProfile(double initialPosition, double finalPosition, double maxVelocity, double maxAcceleration) {
        this.initialPosition = initialPosition;
        this.finalPosition = finalPosition;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;

        generateProfile();
    }

    private void generateProfile() {
        double distance = finalPosition - initialPosition;
        double accelTime = maxVelocity / maxAcceleration;
        double accelDistance = 0.5 * maxAcceleration * accelTime * accelTime;

        if (2 * accelDistance > Math.abs(distance)) {
            // Triangle profile (max velocity not reached)
            accelTime = Math.sqrt(Math.abs(distance) / maxAcceleration);
            maxVelocity = maxAcceleration * accelTime;
            t1 = accelTime;
            t2 = 0;
            t3 = accelTime;
        } else {
            // Trapezoidal profile
            t1 = accelTime;
            t2 = (Math.abs(distance) - 2 * accelDistance) / maxVelocity;
            t3 = accelTime;
        }
        totalTime = t1 + t2 + t3;
    }

    public MotionState getMotionState(double time) {
        double position, velocity, acceleration;
        double distance = finalPosition - initialPosition;
        int direction = (distance >= 0) ? 1 : -1;

        if (time <= t1) {
            // Acceleration phase
            acceleration = direction * maxAcceleration;
            velocity = acceleration * time;
            position = initialPosition + 0.5 * acceleration * time * time;
        } else if (time <= (t1 + t2)) {
            // Constant velocity phase
            acceleration = 0;
            velocity = direction * maxVelocity;
            position = initialPosition + direction * (0.5 * maxAcceleration * t1 * t1 + maxVelocity * (time - t1));
        } else if (time <= totalTime) {
            // Deceleration phase
            double dt = time - t1 - t2;
            acceleration = -direction * maxAcceleration;
            velocity = direction * maxVelocity + acceleration * dt;
            position = finalPosition - 0.5 * acceleration * dt * dt;
        } else {
            // Movement complete
            acceleration = 0;
            velocity = 0;
            position = finalPosition;
        }

        return new MotionState(position, velocity, acceleration);
    }

    public double getTotalTime() {
        return totalTime;
    }
}

class MotionState {
    private final double position;
    private final double velocity;
    private final double acceleration;

    public MotionState(double position, double velocity, double acceleration) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    public double getPosition() { return position; }
    public double getVelocity() { return velocity; }
    public double getAcceleration() { return acceleration; }
}
