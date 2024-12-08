package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Caching.CachingDcMotorEx;

@Disabled

public class minimumKineticTuner {
    private CachingDcMotorEx[] motors;

    public double[] minPowerToOvercomeStaticFriction;
    public double[] minPowerToOvercomeKineticFriction;

    private static final double POWER_INCREMENT = 0.005;
    private static final int SAMPLES = 20;
    private static final long WAIT_TIME_MS = 60;
    private static final double NOMINAL_VOLTAGE = 12.0;

    public minimumKineticTuner(CachingDcMotorEx[] motors) {
        this.motors = motors;
        this.minPowerToOvercomeStaticFriction = new double[motors.length];
        this.minPowerToOvercomeKineticFriction = new double[motors.length];
    }

    public void calibrate(HardwareMap hardwareMap) {
        double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();;
        double voltageScale = NOMINAL_VOLTAGE / currentVoltage;

        for (int i = 0; i < motors.length; i++) {
            DcMotorEx motor = motors[i];
            double staticFrictionTotal = 0.0;
            double kineticFrictionTotal = 0.0;

            // Repeat the process SAMPLES times to get average values
            for (int sample = 0; sample < SAMPLES; sample++) {
                // Reset motor encoder
                motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

                // Start from zero power
                double power = 0.0;
                boolean isMoving = false;

                // Gradually increase power to overcome static friction
                while (!isMoving && power <= 1.0) {
                    motor.setPower(power * voltageScale);
                    // Wait briefly to allow motor to respond
                    sleep(WAIT_TIME_MS);

                    // Check if motor has started moving
                    double velocity = Math.abs(motor.getVelocity());
                    if (velocity > thresholdVelocity()) {
                        isMoving = true;
                        staticFrictionTotal += power;
                    } else {
                        power += POWER_INCREMENT;
                    }
                }

                while (isMoving && power >= 0.0) {
                    power -= POWER_INCREMENT;
                    motor.setPower(power * voltageScale);
                    sleep(WAIT_TIME_MS);

                    double velocity = Math.abs(motor.getVelocity());
                    if (velocity < thresholdVelocity()) {
                        isMoving = false;
                        kineticFrictionTotal += power;
                    }
                }

                motor.setPower(0.0);
            }

            minPowerToOvercomeStaticFriction[i] = (staticFrictionTotal / SAMPLES) * voltageScale;
            minPowerToOvercomeKineticFriction[i] = (kineticFrictionTotal / SAMPLES) * voltageScale;
        }
    }

    private void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private double thresholdVelocity() {
        return 5;
    }
}
