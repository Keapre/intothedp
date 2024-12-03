package org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Caching.CachingDcMotorEx;

import org.firstinspires.ftc.teamcode.Utils.Utils;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.Encoder;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.WEncoder;

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

        public static double kP = 0.002;
        public static double kI = 0.0;
        public static double kD = 0.000;
        public static double kF = 0.0;
        public static double tickPerInch = 0.0;

    public double power = 0;
    public double target = 0;
    public double currentPos = 0;
    public double offset = 0;
    public double currentLength = 0;
    public static double kCos = 0.3;
    public MODE mode = MODE.AUTO;
    boolean usePid = false;
    public PIDFController controller = new PIDFController(kP,0,kD,0);

    public static int sign = 1;
    Encoder encoder;
    Arm arm;
    public Extension(HardwareMap hardwareMap,boolean isAuto,Arm arm) {
        this.arm = arm;
        motor = hardwareMap.get(DcMotorEx.class, "extend");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        encoder = new Encoder(motor);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        offset = encoder.getCurrentPosition();
    }

    public void setPower(double power) {
        this.power = power;
    }
    public void pidUpdate() {
        controller.setPIDF(kP, kI, kD, kF);

        double error = target - (encoder.getCurrentPosition() - offset);
        power = controller.calculate(encoder.getCurrentPosition()-offset,target);
        power*=-1;
    }


    public void manualControl(double power) {
        this.power = power;
        mode = MODE.MANUAL;
    }
    public double getTrueCurrentPosition() {
        return encoder.getCurrentPosition();
    }

    public double getCurrentPos() {
        return getTrueCurrentPosition() - offset;
    }
    public boolean isAtZero() {
        return Math.abs(target-currentPos) < 500;
    }

    public boolean isRetracted() {
        return Math.abs(currentPos-offset) <=650;
    }
    public boolean isAtPosition(double position) {
        return Math.abs(currentPos - position) < 50;
    }

    public double getCurrentPosition() {
        return motor.getCurrentPosition();
    }
    public void update() {
        currentPos = getCurrentPos();
        currentLength = currentPos /tickPerInch;
        double angle = arm.pitchSubsystem.calculateAngle();

        double ff = Math.sin(arm.pitchSubsystem.calculateAngle()) * kCos;
        ff = Utils.minMaxClip(ff,0,1);
        ff*=-1;
        switch (mode) {
            case AUTO:
                mode = MODE.AUTO;
                pidUpdate();

                motor.setPower(Utils.minMaxClip(power + ff,-0.95,0.95));
                break;
            case MANUAL:
                motor.setPower(Utils.minMaxClip(-1,1,power));
                break;
            case IDLE:
                motor.setPower(-0.05);
                break;
        }
    }

}