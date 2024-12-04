package org.firstinspires.ftc.teamcode.subsystems.Arm;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Utils.Utils;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.Encoder;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.profile.ProfileState;

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
        public static double kP = 0.013;
        public static double kI = 0.0;
        public static double kD = 0.00025;

        public static double tickPerDegree = 6.10352;

        public static double Kcos = 0.2;
    }

    private long lastUpdateTime;
    // Motion profile parameters
    public static double maxVelocity = 5000; // degrees per second
    public static double maxAcceleration = 5500; // degrees per second squared
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

    public  double ff = 0;
    public static double angle = 0;

    DigitalChannel limitSwitch;
    public  double offset = 0;
    InterpLUT lut = null;
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
    AsymmetricMotionProfile profile;
    ProfileConstraints constraints;
    ProfileState state;
    private Encoder encoder;
    Arm arm;
    public static double gardFeedforwd = 0.1;
    public static double specimenFeedforwd = 0.115;
    public static double basketFeedforwd = 0;

    AsymmetricMotionProfile motionProfile;
    public Pitch(HardwareMap hardwareMap,boolean isAutonomous,Arm arm) {
        extension2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"pivot2"),0.0);
        extension1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"pivot1"),0.0);

        this.arm = arm;
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limit");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        extension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder = new Encoder(extension1);
        initializeMotors();

        if(isAutonomous) {
            mode = MODE.AUTO;
        }
        currentPos = encoder.getCurrentPosition();
        offset = currentPos;
        maxVelocityCounts = maxVelocity * tickperDegree;
        maxAccelerationCounts = maxAcceleration * tickperDegree/* calculated max acceleration in counts/sec^2 */;
        constraints = new ProfileConstraints(maxAccelerationCounts,maxVelocityCounts, maxAccelerationCounts);
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



    public boolean isAtPosition(double target) {
        return Math.abs(target -currentPos) <= 12;
    }

    public double desiredVelocity = 0,desiredPosition = 0;
    public void setTarget(double pos) {
        target = pos;
    }
    public double calculateAngle() {
        return currentPos / Params.tickPerDegree;
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
        Log.d("MotionProfileA",  "time " + timer.time());
        Log.d("MotionProfileA","total Time" + motionProfile.totalTime);
        state = motionProfile.calculate(timer.time());
        Log.d("MotionProfileA ",motionProfile.toString());
        desiredPosition = state.x;
        Log.d("desired pos","desired " + desiredPosition);
        desiredVelocity = state.v;
        Log.d("pos", String.valueOf(targetPosition));
        Log.d("Velocity",desiredVelocity + "");
        motor1Power = controller.calculate(currentPos,desiredPosition);
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
        motionProfileTime = 0.0;
        timer = new ElapsedTime();
        motionProfile = new AsymmetricMotionProfile(motionProfileStartPosition,motionProfileEndPosition,constraints);
        isMotionProfileActive = true;
        Log.d("MotionProfileA", "Starting motion profile:");
        Log.d("MotionProfileA", "Start Position: " + motionProfileStartPosition);
        Log.d("MotionProfileA", "End Position: " + motionProfileEndPosition);
        Log.d("MotionProfileA", "Distance: " + motionProfile.distance);
        Log.d("MotionProfileA", "Total Time: " + motionProfile.totalTime);

    }

    public void update() {
        checkForSwitch();
        currentPos = encoder.getCurrentPosition() - offset;
        angle = calculateAngle();

        switch ((int) target) {
            case 0:
                ff = 0;
                break;
            case 95:
                ff = gardFeedforwd;
                break;
            case 230:
                ff = specimenFeedforwd;
                break;
            case 320:
                ff = specimenFeedforwd;
                break;
            case 555:
                ff = basketFeedforwd;
                break;
        }
        ff = Math.cos(Math.toRadians(angle)) * gardFeedforwd;
        switch (mode) {
            case AUTO:
                pid();
//                if(target==0 && isAt0()) {
//                    extension2.setPower(0);
//                    extension1.setPower(0);
//                    mode = MODE.IDLE;
//                }
//                else {
//                    extension1.setPower(Utils.minMaxClip(-1, 1, motor1Power));
//                    extension2.setPower(Utils.minMaxClip(-1, 1, motor2Power));
//                }motionProfilePid();
                extension1.setPower(Utils.minMaxClip(motor1Power + ff, -0.5, 0.6));
                extension2.setPower(Utils.minMaxClip(motor2Power + ff,-0.5, 0.6));
                break;
            case MANUAL:
                extension1.setPower(Utils.minMaxClip(-1,1,motor1Power + ff));
                extension2.setPower(Utils.minMaxClip(-1,1,motor2Power + ff));
                break;
            case IDLE:
//                if(lut == null && target == 320) {
//                    lut = new InterpLUT();
//                    lut.add(arm.extensionSubsystem.getCurrentPosition(),specimenFeedforwd);
//                    lut.add(arm.extensionSubsystem.currentPos + 500,0.22);
//                    lut.createLUT();
//                }
//                if(target!=320) {
//                    lut = null;
//                }else {
//                    ff = lut.get(arm.extensionSubsystem.currentPos);
//                }
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
