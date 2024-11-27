package org.firstinspires.ftc.teamcode.subsystems.Arm;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Utils.Utils;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.Encoder;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.WEncoder;

@Config
public class Pitch {

    public enum PITCHPOS {
        UP(0),
        HIGH_BASKET(560),
        LOW_BASKET(560),

        HIGH_CHAMBER(226),
        LOW_CHAMBER(80),

        DOWN(0);

        public double position;
        PITCHPOS(double x) {
            position = x;
        }
    }

    public enum MODE {
        MANUAL,
        AUTO,
        IDLE
    }
    public static double kV = 0.05;
    public static class Params {
        public static double kP = 0.01;
        public static double kI = 0.0;
        public static double kD = 0.00025;

        public static double tickPerDegree = 6.10352;

        public static double Kcos = 0.16;
    }

    private long lastUpdateTime;
    // Motion profile parameters
    public static double maxVelocity = 1000; // degrees per second
    public static double maxAcceleration = 3500; // degrees per second squared
    private double dt = 0.02; // Time step in seconds (50 Hz update rate)

    // Motion profile variables
    public boolean isMotionProfileActive = false;
    private double motionProfileTime = 0.0;
    private double motionProfileTotalTime = 0.0;
    private double motionProfileStartPosition = 0.0;
    private double motionProfileEndPosition = 0.0;

    double threeshouldO = 20;
    //Params PARAMS = new Params();
    public double motor1Power = 0;
    public double motor2Power = 0;
    CachingDcMotorEx extension1; // with encoder
    CachingDcMotorEx extension2; // non ecoder

    public PIDController controller = new PIDController(Params.kP, Params.kI, Params.kD);

    public static double ff = 0;
    public static double angle = 0;

    DigitalChannel limitSwitch;
    public  double offset = 0;
    InterpLUT lut = new InterpLUT();
    public static double currentPos = 0;
    public MODE mode = MODE.AUTO;
    public static double kGpowerInceput = 0.05;
    public static double kGpowerFinal = 0.18;

    public String currentPhase = "Acceleration";
    public static double maxVelocityCounts = maxVelocity * Params.tickPerDegree;
    public static double maxAccelerationCounts = maxAcceleration * Params.tickPerDegree/* calculated max acceleration in counts/sec^2 */;
    public double target = 0;

    ElapsedTime timer = new ElapsedTime();
    public boolean isAt0() {
        return Math.abs(currentPos-threeshouldO) < 6;
    }

    public double tickperDegree = 6.10352;
    private Encoder encoder;
    Arm arm;

    AsymmetricMotionProfile motionProfile;
    public Pitch(HardwareMap hardwareMap,boolean isAutonomous,Arm arm) {
        extension2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"pivot2"),0.0);
        extension1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"pivot1"),0.0);

        this.arm = arm;
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limit");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        extension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extension2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder = new Encoder(extension1);
        lut.add(-50,kGpowerInceput);
        lut.add(850,kGpowerFinal);
        lut.createLUT();
        initializeMotors();

        if(isAutonomous) {
            mode = MODE.AUTO;
        }
        currentPos = encoder.getCurrentPosition();
        offset = currentPos;
        maxVelocityCounts = maxVelocity * tickperDegree;
        maxAccelerationCounts = maxAcceleration * tickperDegree/* calculated max acceleration in counts/sec^2 */;
    }

    public void initializeMotors() {
        extension2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extension1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        extension1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extension2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //REVERSE IF NEEDED
    }

    public double getTrueCurrentPosition() {
        return encoder.getCurrentPosition();
    }

    public double getCurrentPos() {
        return getTrueCurrentPosition() - offset;
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }
    public boolean atTarget() {
        if(mode == MODE.AUTO) {
            if(controller.atSetPoint()) return true;
            else return false;
        }
        return false;
    }
    public boolean isAtPosition(double position) {
        double targetCounts = position ;
        return Math.abs(position -currentPos) < (2 * tickperDegree); // Within 2 degrees
    }

    public double desiredVelocity = 0,desiredPosition = 0;
    public void setTarget(double pos) {
        target = pos;
    }
    public double calculateAngle() {
        return currentPos / Params.tickPerDegree;
    }

    public void setPowerMotors(double power) {
        this.motor1Power = power;
        this.motor2Power = power;
    }

    public void checkForSwitch() {
        if(limitSwitch.getState()) {

            offset = getTrueCurrentPosition();
        }
    }
    public void pid() {
        controller.setSetPoint(target);
        mode = MODE.AUTO;
        controller.setPID(Params.kP, Params.kI, Params.kD);

        motor1Power = controller.calculate(currentPos,target);
        motor2Power = motor1Power;
    }

    public void motionProfilePid() {
        double targetPosition = target;
        long currentTimeMillis = System.currentTimeMillis();
        double deltaTime = (currentTimeMillis - lastUpdateTime) / 1000.0; // Convert to seconds
        lastUpdateTime = currentTimeMillis;
        Log.d("time Delta",motionProfileTotalTime + " " + motionProfileTime + " " + timer.seconds());
        if (isMotionProfileActive) {
            motionProfileTime += deltaTime;
            if (timer.seconds() >= motionProfileTotalTime) {
                motionProfileTime = motionProfileTotalTime;
                isMotionProfileActive = false;
            }

            generateMotionProfileState(motionProfileTime);

            Log.d("pos", String.valueOf(targetPosition));
        }
        Log.d("Pitch Pid",targetPosition + " " + isMotionProfileActive + " " + motionProfileTime);


        Log.d("Velocity",desiredVelocity + "");
        motor1Power = controller.calculate(currentPos,desiredPosition) + kV * desiredVelocity;
        motor2Power = motor1Power;

    }

    public MODE getMode() {
        return mode;
    }

    public void setMode(MODE md) {
        this.mode = md;
    }
    public void startMotionProfile(double endPositionDegrees) {

        motionProfileStartPosition = currentPos;
        motionProfileEndPosition = endPositionDegrees;
        double distance = motionProfileEndPosition - motionProfileStartPosition;
        motionProfileTotalTime = calculateMotionProfileTotalTime(distance);
        Log.d("time",motionProfileTotalTime + " " + motionProfileStartPosition + " " + motionProfileEndPosition);
        motionProfileTime = 0.0;
        timer.reset();
        timer.startTime();
        isMotionProfileActive = true;
        Log.d("Pitch", "Starting motion profile:");
        Log.d("Pitch", "Start Position: " + motionProfileStartPosition);
        Log.d("Pitch", "End Position: " + motionProfileEndPosition);
        Log.d("Pitch", "Distance: " + distance);
        Log.d("Pitch", "Total Time: " + motionProfileTotalTime);

    }

    private double calculateMotionProfileTotalTime(double distanceDegrees) {
        double distance = Math.abs(distanceDegrees);
        double timeToMaxVelocity = maxVelocityCounts / maxAccelerationCounts;

        double distanceDuringAccel = 0.5 * maxAccelerationCounts * timeToMaxVelocity * timeToMaxVelocity;

        double totalTime = 0;
        if (distance < 2 * distanceDuringAccel) {
            // Triangle profile (never reaches max velocity)
            totalTime =  2 * Math.sqrt(distance / maxAccelerationCounts);
        } else {
            // Trapezoidal profile
            double distanceAtConstantVelocity = distance - (2 * distanceDuringAccel);
            double timeAtConstantVelocity = distanceAtConstantVelocity / maxVelocityCounts;
            totalTime = 2 * timeToMaxVelocity + timeAtConstantVelocity;
        }
        Log.d("Pitch", "Calculated Motion Profile Total Time: " + totalTime);
        return totalTime;
    }

    private void generateMotionProfileState(double time) {
        double distance = motionProfileEndPosition - motionProfileStartPosition;
        double direction = Math.signum(distance);
        double absDistance = Math.abs(distance);

        double timeToMaxVelocity = maxVelocityCounts / maxAccelerationCounts;
        double distanceDuringAccel = 0.5 * maxAccelerationCounts * timeToMaxVelocity * timeToMaxVelocity;

        double totalTime;

        if (absDistance < 2 * distanceDuringAccel) {
            // Triangle profile
            timeToMaxVelocity = Math.sqrt(absDistance / maxAccelerationCounts);
            totalTime = 2 * timeToMaxVelocity;

            if (time < timeToMaxVelocity) {
                // Acceleration phase
                desiredPosition = motionProfileStartPosition + direction * 0.5 * maxAccelerationCounts * time * time;
                desiredVelocity = direction * maxAccelerationCounts * time;
                currentPhase = "Acceleration";
            } else if (time <= totalTime) {
                // Deceleration phase
                double t = time - timeToMaxVelocity;
                desiredPosition = motionProfileStartPosition + direction * (absDistance - 0.5 * maxAccelerationCounts * t * t);
                desiredVelocity = direction * maxAccelerationCounts * (totalTime - time);
                currentPhase = "Deceleration";
            } else {
                desiredPosition = motionProfileEndPosition;
                desiredVelocity = 0.0;
                currentPhase = "Completed";
            }
        } else {
            // Trapezoidal profile
            double timeAtConstantVelocity = (absDistance - (2 * distanceDuringAccel)) / maxVelocityCounts;
            totalTime = 2 * timeToMaxVelocity + timeAtConstantVelocity;

            if (time < timeToMaxVelocity) {
                // Acceleration phase
                desiredPosition = motionProfileStartPosition + direction * 0.5 * maxAccelerationCounts * time * time;
                desiredVelocity = direction * maxAccelerationCounts * time;
                currentPhase = "Acceleration";
            } else if (time < (timeToMaxVelocity + timeAtConstantVelocity)) {
                // Constant velocity phase
                double accelDistance = distanceDuringAccel;
                double constantVelocityTime = time - timeToMaxVelocity;
                desiredPosition = motionProfileStartPosition + direction * (accelDistance + maxVelocityCounts * constantVelocityTime);
                desiredVelocity = direction * maxVelocityCounts;
                currentPhase = "Constant Velocity";
            } else if (time <= totalTime) {
                // Deceleration phase
                double t = time - timeToMaxVelocity - timeAtConstantVelocity;
                desiredPosition = motionProfileEndPosition - direction * 0.5 * maxAccelerationCounts * t * t;
                desiredVelocity = direction * maxAccelerationCounts * (totalTime - time);
                currentPhase = "Deceleration";
            } else {
                desiredPosition = motionProfileEndPosition;
                desiredVelocity = 0.0;
                currentPhase = "Completed";
            }
        }

        // Logging
        Log.d("MotionProfile", "Time: " + time);
        Log.d("MotionProfile", "Phase: " + currentPhase);
        Log.d("MotionProfile", "Desired Position: " + desiredPosition);
        Log.d("MotionProfile", "Desired Velocity: " + desiredVelocity);
    }



    public void update() {
        checkForSwitch();
        currentPos = encoder.getCurrentPosition() - offset;
        angle = calculateAngle();
//        ff = Math.cos(Math.toRadians(angle)) * lut.get(Utils.minMaxClip(arm.extensionSubsystem.getCurrentPos(),0,699));
        ff = Math.cos(Math.toRadians(angle)) * Params.Kcos;
        switch (mode) {
            case AUTO:
//                pid();
//                if(target==0 && isAt0()) {
//                    extension2.setPower(0);
//                    extension1.setPower(0);
//                    mode = MODE.IDLE;
//                }
//                else {
//                    extension1.setPower(Utils.minMaxClip(-1, 1, motor1Power));
//                    extension2.setPower(Utils.minMaxClip(-1, 1, motor2Power));
//                }
                motionProfilePid();
                if (isMotionProfileActive) {
                    Log.d("Pitch", "Motion Profile Time: " + motionProfileTime);
                    Log.d("Pitch", "Target Position: " + target);
                    Log.d("Pitch", "Current Position: " + currentPos);

                    // ... existing code ...
                } else {
                    mode = MODE.IDLE;

                    Log.d("Pitch", "Motion Profile is not active.");
                    break;
                }
                extension1.setPower(Utils.minMaxClip(-1, 1, motor1Power + ff));
                extension2.setPower(Utils.minMaxClip(-1, 1, motor2Power + ff));
                break;
            case MANUAL:
                extension1.setPower(Utils.minMaxClip(-1,1,motor1Power + ff));
                extension2.setPower(Utils.minMaxClip(-1,1,motor2Power + ff));
                break;
            case IDLE:
                if(target == 0) {
                    extension1.setPower(0);
                    extension2.setPower(0);
                }
                else {
                    extension1.setPower(ff);
                    extension2.setPower(ff);
                }
                break;
        }
    }


}

