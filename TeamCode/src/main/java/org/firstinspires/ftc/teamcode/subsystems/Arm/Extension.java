package org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Utils.Caching.CachingDcMotorEx;

import org.firstinspires.ftc.teamcode.Utils.GameStatitics.Timer;
import org.firstinspires.ftc.teamcode.Utils.Utils;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.Encoder;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.WEncoder;

@Config
public class Extension {

   public  DcMotorEx motor;
    public enum EXTENSIONLENGTH {
        MIN(0),

        STAGE1(0),
        STAGE2(0),
        STAGE3(0),
        MAX(0);

        public double length;
        EXTENSIONLENGTH(double x) {
            length = x;
        }

        public double getLength(){
            return length;
        }
    }

    public static double retractedThreeshold = 15;
    public static double pointThreeshold = 20;

    public enum MODE {
        MANUAL,
        RAW_POWER,
        AUTO,
        IDLE
    }

        public static double kP = 0.04;
        public static double kI = 0.0;
        public static double kD = 0.0007;
        public static double kF = 0.0;
        public static double tickPerInch = 0.0;

    public double power = 0;
    public double target = 0;
    public double currentPos = 0;
    public double offset = 0;
    public double currentLength = 0;
    public static double kCos = 0.03;
    public static double basePower = 0;
    public MODE mode = MODE.AUTO;
    boolean usePid = false;
    public PIDFController controller = new PIDFController(kP,0,kD,0);

    public static int sign = 1;
    Encoder encoder;
    Arm arm;
    ElapsedTime timer = null;
    double valueTimer = 0;
    public Extension(HardwareMap hardwareMap,boolean isAuto,Arm arm) {
        this.arm = arm;
        motor = hardwareMap.get(DcMotorEx.class, "extend");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        encoder = new Encoder(motor);
        encoder.setDirection(Encoder.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        offset = encoder.getCurrentPosition();
    }

    public static double raw_power = 0;

    public void changeRawPower(double power) {
        raw_power = power;
    }
    double previous_angle = 0;
    public void setPower(double power) {
        this.power = power;
    }
    public void pidUpdate() {
        controller.setPIDF(kP, kI, kD, kF);

        double error = target - (encoder.getCurrentPosition() - offset);
        power = controller.calculate(currentPos,target);

    }

    public double ff = 0;


    public void manualControl(double power) {
        this.power = power;
        mode = MODE.MANUAL;
    }

    public double getPosition() {
        return currentPos;
    }
    public boolean isAtZero() {
        return Math.abs(target-currentPos) < 500;
    }

    public boolean isRetracted() {
        return Math.abs(currentPos-target) <=retractedThreeshold;
    }
    public boolean isAtPosition(double position) {
        return Math.abs(currentPos - position) < pointThreeshold;
    }
    public static double valueSHit = 36;

    public double getTimer() {
        if(timer==null) return 0;
        return timer.time();
    }
    public double getCurrentPosition() {
        return currentPos;
    }
    public double getTrueCurrentPosition() {
        return encoder.getCurrentPosition();
    }

    public double getCurrentPos(double angle) {
        return getTrueCurrentPosition() - offset - (Math.sin(Math.toRadians(angle)) * valueSHit);
    }

    public void update() {
        double angle = arm.pitchSubsystem.calculateAngle();
        currentPos = getCurrentPos(angle);
        if(arm.currentState == Arm.FSMState.RETRACTING_EXTENSION || arm.currentState == Arm.FSMState.EXTENDING_EXTENSION) {
            if(timer == null) {
                timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                valueTimer = currentPos;
            }
            if(Math.abs(currentPos-valueTimer)>4){
                timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                valueTimer = currentPos;
            }
        }
        else {
            timer = null;
        }
        switch (mode) {
            case AUTO:
                raw_power = 0;
                mode = MODE.AUTO;
                pidUpdate();
                power*=-1;
                power+=ff;
                motor.setPower(Utils.minMaxClip(power - basePower,-1, 0.75));
                break;
            case MANUAL:
                raw_power = 0;
                motor.setPower(Utils.minMaxClip(-1,1,power + basePower));
                break;
            case RAW_POWER:
                motor.setPower(raw_power);
                break;
            case IDLE:
                raw_power = 0;
                if(arm.targetState.getPitchAngle() == 555) {
                    motor.setPower(-0.05);
                }
                else {
                    motor.setPower(0);
                }
                break;
        }
        previous_angle = angle;
    }

}