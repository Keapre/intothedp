package org.firstinspires.ftc.teamcode.subsystems.Arm.Pitch;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Utils.Utils;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.Encoder;


public class Pitch {
    public static boolean IS_DISABLED = false;
    private Robot robot;

    CachingDcMotorEx extension2; // non ecoder
    CachingDcMotorEx extension1; // with encoder
    Encoder encoder = null;


    public PIDController controller = new PIDController(PitchConstants.kP, 0, PitchConstants.kD);

    public enum MODE {
        MANUAL,
        AUTO,
        IDLE
    }

    public double motor1Power = 0;
    public double motor2Power = 0;
    public double ff = 0;


    public static double angle = 0;

    DigitalChannel limitSwitch;
    public  double offset = 0;
    InterpLUT lutExtendIntake = null;
    public static double currentPos = 0;
    public MODE mode = MODE.AUTO;

    private double target = 0;



    public Pitch(HardwareMap hardwareMap,boolean isAutonomous,Robot robot) {
        this.robot = robot;
        extension2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"pivot2"),0.0);
        extension1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"pivot1"),0.0);
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limit");
        extension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder = new Encoder(extension1);

        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        initializeMotors();

        if(isAutonomous) {
            mode = MODE.AUTO;
        }
        currentPos = encoder.getCurrentPosition();
        offset = currentPos;
    }

    public void initializeMotors() {
        extension2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extension1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        extension1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extension2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //REVERSE IF NEEDED
    }

    double minnLinerPos = 0,maxxLinearPos = 0;
    public void updateRegreesion() {
        lutExtendIntake = new InterpLUT();
        minnLinerPos = currentPos;
        maxxLinearPos = currentPos + 250;
        lutExtendIntake.add(currentPos-1,1);
        lutExtendIntake.add(currentPos+50,1.2);
        lutExtendIntake.add(currentPos+100,1.2);
        lutExtendIntake.add(currentPos+150,1.4);
        lutExtendIntake.add(currentPos+200,1.6);
        lutExtendIntake.add(currentPos+251,1.8);
        lutExtendIntake.createLUT();
        //TODO:tune this
    }
    public double getTrueCurrentPosition() {
        return encoder.getCurrentPosition();
    }

    public double getCurrentPos() {
        return currentPos;
    }

    public boolean isAtPosition(double target) {
        return Math.abs(target -currentPos) <= PitchConstants.threshold;
    }
    public boolean isAt0() {
        return Math.abs(currentPos-PitchConstants.lowThreshold) < PitchConstants.threshold;
    }

    public void setTarget(double pos)
    {
        controller.reset();
        target = pos;
        mode = MODE.AUTO;
    }
    public double calculateAngle() {
        return currentPos / PitchConstants.tickPerDegree;
    }

    public double get_angle() {
        return angle;
    }
    public void checkForSwitch() {
        if(limitSwitch.getState()) {
            offset = getTrueCurrentPosition();
        }
    }
    public double getTarget() {
        return target;
    }
    public void pid() {
        mode = MODE.AUTO;
        controller.setPID(PitchConstants.kP, 0, PitchConstants.kD);
        motor1Power = controller.calculate(currentPos,target);
        motor1Power = clamp(motor1Power,PitchConstants.low_Value,PitchConstants.max_Value);
        motor2Power = motor1Power;
    }

    public MODE getMode() {
        return mode;
    }

    public void setMode(MODE md) {
        this.mode = md;
    }

    public void update() {
        checkForSwitch();
        currentPos = getTrueCurrentPosition() - offset;
        angle = calculateAngle();
        if(IS_DISABLED) return;
        ff = Math.cos(Math.toRadians(angle)) * PitchConstants.max_f;
        if(lutExtendIntake == null) {
            updateRegreesion();
        }
        ff*=lutExtendIntake.get(clamp(robot.arm.extensionSubsystem.getPosition(),minnLinerPos,maxxLinearPos));
        switch (mode) {
            case AUTO:
                pid();
                extension1.setPower(Utils.minMaxClip((motor1Power + ff) * robot.getNormalizedVoltage(), -1, 1));
                extension2.setPower(Utils.minMaxClip((motor2Power + ff) * robot.getNormalizedVoltage(),-1, 1));
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
                    extension1.setPower(ff * robot.getNormalizedVoltage());
                    extension2.setPower(ff * robot.getNormalizedVoltage());
                }
                break;
        }
    }


}