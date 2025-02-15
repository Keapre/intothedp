package org.firstinspires.ftc.teamcode.Utils;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Mecanum wheel drive calculations.
 * Input controls:
 * V_d = desired robot speed.
 * theta_d = desired robot velocity angle.
 * V_theta = desired robot rotational speed.
 * <p>
 * Example:
 * // Convert joysticks to wheel powers.
 * Mecanum.Wheels wheels = Mecanum.motionToWheels(
 * Mecanum.joystickToMotion(
 * gamepad1.left_stick_x, gamepad1.left_stick_y,
 * gamepad1.right_stick_x, gamepad1.right_stick_y));
 * // Set power on the motors.
 * frontLeftMotor.setPower(wheels.frontLeft);
 */
public class MecanumUtil {
    /**
     * Mecanum motion vector.
     */
    public static class Motion {
        // Robot speed [-1, 1].
        public final double vD;
        // Robot angle while moving [0, 2pi].
        public final double thetaD;
        // Speed for changing direction [-1, 1].
        public final double vTheta;

        /**
         * Sets the motion to the given values.
         */
        public Motion(double vD, double thetaD, double vTheta) {
            this.vD = vD;
            this.thetaD = thetaD;
            this.vTheta = vTheta;
        }

        /**
         * Transforms a robot centric motion to a field centric motion.
         *
         * @param robotTheta The angle of the robot.
         * @return The Mecanum motion vector.
         */
        public Motion toFieldCentricMotion(double robotTheta) {
            return new Motion(vD, thetaD - robotTheta, vTheta);
        }
    }

    /**
     * Apply custom curve for each parameter
     * @param x
     * @return
     */
    public static double applyCurve(double x) {
        return x * x * Math.signum(x);
    }

    /**
     * Gets the motion vector from the joystick values.
     *
     * @param leftStickX  The left joystick X.
     * @param leftStickY  The left joystick Y.
     * @param rightStickX The right joystick X.
     * @param rightStickY The right joystick Y.
     * @param customCurve Apply the custom curve or not
     * @return The Mecanum motion vector.
     */
    public static Motion joystickToMotion(double leftStickX,
                                          double leftStickY,
                                          double rightStickX,
                                          double rightStickY,
                                          boolean reverseFront,
                                          boolean customCurve) {
        if (customCurve) {
            leftStickX = applyCurve(leftStickX);
            leftStickY = applyCurve(leftStickY);
            rightStickX = applyCurve(rightStickX);
            rightStickY = applyCurve(rightStickY);
        }

        double vD = Math.min(Math.sqrt(Math.pow(leftStickX, 2) +
                        Math.pow(leftStickY, 2)),
                1);
        if(reverseFront) {
            leftStickX = -leftStickX;
            leftStickY = -leftStickY;
        }
        double thetaD = Math.atan2(-leftStickX, -leftStickY);
        double vTheta = -rightStickX;
        return new Motion(vD, thetaD, vTheta);
    }

    public static Motion motionFromPowerVector(double x,double y,double h) {
        double vD = Math.min(Math.sqrt(Math.pow(x, 2) +
                        Math.pow(y, 2)),
                1);
        double thetaD = Math.atan2(-x, -y);
        double vTheta = h;
        return new Motion(vD, thetaD, vTheta);
    }

    /**
     * Mecanum wheels, used to get individual motor powers.
     */
    public static class Wheels {
        // The mecanum wheels.
        public final double frontLeft;
        public final double frontRight;
        public final double backLeft;
        public final double backRight;

        /**
         * Sets the wheels to the given values.
         */
        public Wheels(double frontLeft, double frontRight,
                      double backLeft, double backRight) {
            List<Double> powers = Arrays.asList(frontLeft, frontRight,
                    backLeft, backRight);
            clampPowers(powers);

            this.frontLeft = powers.get(0);
            this.frontRight = powers.get(1);
            this.backLeft = powers.get(2);
            this.backRight = powers.get(3);
        }

        /**
         * Scales the wheel powers by the given factor.
         *
         * @param scalar The wheel power scaling factor.
         */
        public Wheels scaleWheelPower(double scalar) {
            return new Wheels(frontLeft * scalar, frontRight * scalar,
                    backLeft * scalar, backRight * scalar);
        }

        private double applyEquation(double power,double voltage,double ks) {
            return (voltage * power) + (ks * Math.signum(power) * voltage);
        }
        public Wheels scaleAutoPowers(double voltage,double ks) {
            return new Wheels(applyEquation(frontLeft,voltage,ks),applyEquation(frontRight,voltage,ks),
                    applyEquation(backLeft,voltage,ks),applyEquation(backRight,voltage,ks));

        }
    }

    /**
     * Gets the wheel powers corresponding to desired motion.
     *
     * @param motion The Mecanum motion vector.
     * @return The wheels with clamped powers. [-1, 1]
     */
    public static Wheels motionToWheels(Motion motion) {
        double vD = motion.vD;
        double thetaD = motion.thetaD;
        double vTheta = motion.vTheta;

        double frontLeft = vD * Math.sin(-thetaD + Math.PI / 4) - vTheta;
        double frontRight = vD * Math.cos(-thetaD + Math.PI / 4) + vTheta;
        double backLeft = vD * Math.cos(-thetaD + Math.PI / 4) - vTheta;
        double backRight = vD * Math.sin(-thetaD + Math.PI / 4) + vTheta;
        return new Wheels(frontLeft, frontRight,
                backLeft, backRight);
    }

    /**
     * Gets the wheel powers corresponding to desired motion.
     *
     * @param motion The Mecanum motion vector.
     * @return The wheels with clamped powers. [-1, 1]
     */
    public static Wheels motionToWheelsFullSpeed(Motion motion) {
        double vD = motion.vD;
        double thetaD = motion.thetaD;
        double vTheta = motion.vTheta;

        double frontLeft = vD * Math.sin(thetaD + Math.PI / 2) + vD * Math.cos(thetaD + Math.PI / 2) - vTheta;
        double frontRight = vD * Math.sin(thetaD + Math.PI / 2) - vD * Math.cos(thetaD + Math.PI / 2) + vTheta;
        double backLeft = vD * Math.sin(thetaD + Math.PI / 2) - vD * Math.cos(thetaD + Math.PI / 2) - vTheta;
        double backRight = vD * Math.sin(thetaD + Math.PI / 2) + vD * Math.cos(thetaD + Math.PI / 2) + vTheta;
        return new Wheels(frontLeft, frontRight,
                backLeft, backRight);
    }

    /**
     * Clamps the motor powers while maintaining power ratios.
     *
     * @param powers The motor powers to clamp.
     */
    private static void clampPowers(List<Double> powers) {
        double minPower = Collections.min(powers);
        double maxPower = Collections.max(powers);
        double maxMag = Math.max(Math.abs(minPower), Math.abs(maxPower));

        if (maxMag > 1.0) {
            for (int i = 0; i < powers.size(); i++) {
                powers.set(i, powers.get(i) / maxMag);
            }
        }
    }
}