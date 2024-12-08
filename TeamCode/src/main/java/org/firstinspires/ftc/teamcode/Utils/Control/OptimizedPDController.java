package org.firstinspires.ftc.teamcode.Utils.Control;

import java.util.LinkedList;
import java.util.Queue;

public class OptimizedPDController {
    private double kp;
    private double kd;
    private double tau; // Time constant for the low-pass filter on the derivative term
    private double minOutput;
    private double maxOutput;

    private double prevTime;
    private double derivative;
    private RingBuffer errorBuffer;

    public OptimizedPDController(double kp, double kd, double minOutput, double maxOutput, int bufferSize) {
        this.kp = kp;
        this.kd = kd;
        this.tau = 0.07;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        this.prevTime = -1;
        this.derivative = 0.0;
        this.errorBuffer = new RingBuffer(bufferSize);
    }

    public void reset() {
        prevTime = -1;
        derivative = 0.0;
        errorBuffer.clear();
    }

    public void setPid(double kP,double kD) {
        this.kp = kP;
        this.kd = kD;
    }
    public double compute(double setpoint, double measuredValue) {
        double currentTime = System.currentTimeMillis() / 1000.0; // Convert to seconds
        double error = setpoint - measuredValue;
        double dt = (prevTime < 0) ? 0 : currentTime - prevTime;

        // Proportional term
        double pTerm = kp * error;

        // Update the ring buffer with the current error
        errorBuffer.add(error);

        // Derivative term with low-pass filtering using the ring buffer
        if (dt > 0 && errorBuffer.isFull()) {
            double oldestError = errorBuffer.getOldest();
            double derivativeRaw = (error - oldestError) / (errorBuffer.size() * dt);
            derivative = (tau * derivative + derivativeRaw) / (tau + dt);
        }

        double dTerm = kd * derivative;

        // Calculate output and apply square scaling
        double output = pTerm + dTerm;
        output = Math.sqrt(output);

        // Apply output limits (anti-windup)
        if (output > maxOutput) {
            output = maxOutput;
        } else if (output < minOutput) {
            output = minOutput;
        }

        // Save the current time for the next iteration
        prevTime = currentTime;

        return output;
    }

    // Inner class for the ring buffer
    private class RingBuffer {
        private Queue<Double> buffer;
        private int maxSize;

        public RingBuffer(int size) {
            this.maxSize = size;
            this.buffer = new LinkedList<>();
        }

        public void add(double value) {
            if (buffer.size() >= maxSize) {
                buffer.poll(); // Remove the oldest value
            }
            buffer.add(value);
        }

        public double getOldest() {
            return buffer.peek();
        }

        public int size() {
            return buffer.size();
        }

        public boolean isFull() {
            return buffer.size() == maxSize;
        }

        public void clear() {
            buffer.clear();
        }
    }
}
