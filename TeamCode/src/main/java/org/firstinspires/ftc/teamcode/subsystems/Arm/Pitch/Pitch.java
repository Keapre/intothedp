package org.firstinspires.ftc.teamcode.subsystems.Arm.Pitch;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Utils.Utils;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.Debouncer;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.Encoder;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Extension.ExtensionConstants;

@Config

public class Pitch {
    public static boolean IS_DISABLED = false;
    private Robot robot;

    public CachingDcMotorEx extension2; // non ecoder
    public CachingDcMotorEx extension1; // with encoder
    Encoder encoder = null;


    PIDController controller = new PIDController(PitchConstants.kP, 0, PitchConstants.kD);
    PIDController holdingController = new PIDController(PitchConstants.holdingkP, 0, PitchConstants.holdingkD);

    Debouncer debouncer;
    public enum MODE {
        MANUAL,
        AUTO,
        IDLE,
        RAW_POWER
    }

    public double motor1Power = 0;
    public double motor2Power = 0;
    public double ff = 0;


    public  double angle = 0;

    DigitalChannel limitSwitch;
    public  double offset = 0;
    InterpLUT lutExtendIntake = null;
    public  double currentPos = 0;
    public MODE mode = MODE.AUTO;

    public double target = 0;



    public Pitch(HardwareMap hardwareMap,boolean isAutonomous,Robot robot) {
        this.robot = robot;
        extension2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"pivot1"),0.0);
        extension1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"pivot2"),0.0);
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limit");
        extension1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        debouncer =  new Debouncer(0.1, Debouncer.DebounceType.kBoth);;
        encoder = new Encoder(extension1);
        encoder.setDirection(Encoder.Direction.REVERSE);

        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        initializeMotors();
        resetVar();
        if(isAutonomous) {
            mode = MODE.AUTO;
        }
        currentPos = encoder.getCurrentPosition();
        offset = currentPos;
    }

    void resetVar() {
        motor1Power = 0;
        motor2Power = 0;
        ff = 0;
        angle = 0;
        currentPos = 0;
        target = 0;
        mode = MODE.IDLE;
        controller = new PIDController(PitchConstants.kP, 0, PitchConstants.kD);
        offset = 0;
    }
    public void initializeMotors() {
        extension2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extension1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        extension1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extension2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        extension2.setDirection(DcMotorSimple.Direction.REVERSE);
        extension1.setDirection(DcMotorSimple.Direction.REVERSE);

        //REVERSE IF NEEDED
    }

    public double getTrueCurrentPosition() {
        return extension1.getCurrentPosition();
    }

    public double getCurrentPos() {
        return currentPos;
    }

    public boolean isAtPosition(double target) {
        return Math.abs(target -currentPos) <= PitchConstants.threshold;
    }
    public boolean isAt0() {
        return Math.abs(currentPos- ExtensionConstants.at0positionPitch) < ExtensionConstants.at0Threeshold;
    }

    public void setTarget(double pos)
    {
        //controller.reset();
        target = pos;
        mode = MODE.AUTO;
    }
    public double calculateAngle() {
        return currentPos / PitchConstants.tickPerDegree;
    }

    public double get_angle() {
        return angle;
    }
    public boolean checkForSwitch() {
        return debouncer.calculate(limitSwitch.getState());
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
    private double raw_power = 0;
    public void changeRawPower(double target) {
        this.raw_power = target;
    }
    public MODE getMode() {
        return mode;
    }

    public void setMode(MODE md) {
        this.mode = md;
    }
    public boolean USE_EXTENSTIONFEED = false;
    public void update() {
        if(checkForSwitch()) {
            offset = getTrueCurrentPosition();
        }
        currentPos = getTrueCurrentPosition() - offset;
        angle = calculateAngle();
        if(IS_DISABLED) return;
        ff = Math.cos(Math.toRadians(angle))* PitchConstants.max_f;
        if(USE_EXTENSTIONFEED) {
            ff *= (1 + (clamp(robot.arm.extensionSubsystem.currentPos,0,1000)) * PitchConstants.extensionVar);
        }
//        if(lutExtendIntake == null) {
//            updateRegreesion();
//        }'[
        //ff*=(1 + (600/robot.arm.extensionSubsystem.currentPos));
//        ff*=lutExtendIntake.get(clamp(robot.arm.extensionSubsystem.getPosition(),minnLinerPos,maxxLinearPos));
        switch (mode) {
            case RAW_POWER:
                extension1.setPower(raw_power);
                extension2.setPower(raw_power);
                break;
            case AUTO:
                pid();
                extension1.setPower(Utils.minMaxClip(motor1Power +ff  , -1, 1));
                extension2.setPower(Utils.minMaxClip(motor2Power+ff,-1, 1));
                break;
            case MANUAL:
                extension1.setPower(Utils.minMaxClip(-1,1,motor1Power));
                extension2.setPower(Utils.minMaxClip(-1,1,motor2Power));
                break;
            case IDLE:
                if(target == ExtensionConstants.at0positionPitch) {
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