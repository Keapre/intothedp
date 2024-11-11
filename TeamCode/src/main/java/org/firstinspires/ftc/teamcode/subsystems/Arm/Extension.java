package org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Caching.CachingDcMotorEx;

import org.firstinspires.ftc.teamcode.Utils.Utils;

@Config
public class Extension {

    DcMotorEx motor;
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


    public enum MODE {
        MANUAL,
        AUTO,
        IDLE
    }

    public static class Params {
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;
        public static double kF = 0.0;
        public static double tickPerInch = 0.0;
    }

    public double power = 0;
    public double target = 0;
    public double currentPos = 0;
    public double offset = 0;
    public double currentLength = 0;
    public static double kCos = 0;
    public MODE mode = MODE.AUTO;
    boolean usePid = false;
    public PIDFController controller = new PIDFController(Params.kP,0,Params.kD,0);

    Arm arm;
    public Extension(HardwareMap hardwareMap,boolean isAuto,Arm arm) {
        this.arm = arm;
        motor = hardwareMap.get(DcMotorEx.class, "extend");
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        if(isAuto) mode = MODE.AUTO;
        offset = motor.getCurrentPosition();
    }

    public void setPower(double power) {
        this.power = power;
    }
    public void pidUpdate() {
        controller.setSetPoint(target);
        if(controller.atSetPoint()) {
            mode = MODE.IDLE;
        }
        controller.setPIDF(Params.kP, Params.kI, Params.kD, Params.kF);

        power = controller.calculate(currentLength, target);
    }


    public void manualControl(double power) {
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.power = power;
        mode = MODE.MANUAL;
    }

    public boolean isAtZero() {
        return Math.abs(motor.getCurrentPosition() - offset) < 10;
    }

    public boolean isAtPosition(double position) {
        return Math.abs(motor.getCurrentPosition() - position) < 10;
    }

    public double getCurrentPosition() {
        return motor.getCurrentPosition();
    }
    public void update() {
        currentPos = motor.getCurrentPosition() - offset;
        currentLength = currentPos / Params.tickPerInch;
        double angle = arm.pitchSubsystem.calculateAngle();
        double ff = kCos * Math.toRadians(angle);
        switch (mode) {
            case AUTO:
                pidUpdate();
                motor.setPower(power + ff);
                break;
            case MANUAL:
                motor.setPower(Utils.minMaxClip(-1,1,power + ff));
                break;
            case IDLE:
                motor.setPower(ff);
                break;
            default:
                break;
        }
    }

}
