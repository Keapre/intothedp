package org.firstinspires.ftc.teamcode.subsystems.Arm.Extension;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Utils.Utils;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.Encoder;
import org.firstinspires.ftc.teamcode.Utils.geometry.Path;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Arm;

@Config
public class Extension {
    public static boolean IS_DISABLED = false;

    public CachingDcMotorEx motor;

    public enum MODE {
        MANUAL,
        RAW_POWER,
        AUTO,
        IDLE
    }
    public double power = 0;
    public double target = 0;
    public double currentPos = 0;
    public double offset;

    public MODE mode = MODE.AUTO;
    public double angle = 0;
    private double maxx = 0;
    public PIDFController controller = new PIDFController(ExtensionConstants.kP,0,ExtensionConstants.kD,0);

    Encoder encoder;
    ElapsedTime timer = null;
    double valueTimer = 0;
    Robot robot;
    public Extension(HardwareMap hardwareMap, boolean isAuto, Robot robot) {
        this.robot  =robot;
        motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "extend"),0.02);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encoder = new Encoder(motor);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        offset = encoder.getCurrentPosition();
    }

    public static double raw_power = 0;
    private double currentAmp = 0;

    public void setTaget(double target) {
        mode = MODE.AUTO;
        this.target = target;
    }
    public void changeRawPower(double power) {
        raw_power = power;
    }
    double previous_angle = 0;

    private void updateFeedForward() {
        ff = ExtensionConstants.kf * Math.sin(Math.toRadians(angle));
    }
    public void pidUpdate() {
        controller.setPIDF(ExtensionConstants.kP, 0, ExtensionConstants.kD, 0);
        power = controller.calculate(currentPos,target);
        power+=ff;
    }

    public double ff = 0;


    public void manualControl(double power) {
        this.power = power;
        mode = MODE.MANUAL;
    }

    public double getPosition() {
        return currentPos;
    }

    public boolean  isAtPosition() {
        return Math.abs(currentPos - target) <= ExtensionConstants.pointThreeshold;
    }

    public double getMaxAmps() {
        maxx = Math.max(maxx,currentAmp);
        return maxx ;

    }
    public double getAmp()
    {
        return currentAmp;
    }
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
        return getTrueCurrentPosition() - offset - (Math.sin(Math.toRadians(clamp(angle,0,90)) * ExtensionConstants.valueSHit)) ;
    }

    public void checkTimer() {
        if(robot.arm.currentState == Arm.FSMState.RETRACTING_EXTENSION || robot.arm.currentState == Arm.FSMState.EXTENDING_EXTENSION) {
            if(timer == null) {
                timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                valueTimer = currentPos;
            }
            if(Math.abs(currentPos-valueTimer)>ExtensionConstants.dynamicTimerThreshold){
                timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                valueTimer = currentPos;
            }
        }
        else {
            timer = null;
        }
    }

    public void update() {
        currentAmp = motor.getCurrent(CurrentUnit.AMPS);
        angle = robot.arm.pitchSubsystem.get_angle();
        currentPos = getCurrentPos(angle);
        if(IS_DISABLED) return;
        checkTimer();
        updateFeedForward();
        switch (mode) {
            case AUTO:
                pidUpdate();
                motor.setPower(Utils.minMaxClip(power + ff,-1, 1));
                break;
            case RAW_POWER:
                motor.setPower(raw_power);
                break;
            case MANUAL:
                motor.setPower(Utils.minMaxClip(-1,1,power + ff));
                break;
            case IDLE:
                motor.setPower(ExtensionConstants.idlePower);
                break;
        }
        previous_angle = angle;
    }

}