package org.firstinspires.ftc.teamcode.subsystems.Arm.Extension;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Utils.Utils;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.Debouncer;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.Encoder;
import org.firstinspires.ftc.teamcode.Utils.geometry.Path;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Arm.ArmState;

@Config
public class Extension {
    public boolean IS_DISABLED = false;

    public DcMotorEx motor;

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
     PIDFController controller = new PIDFController(ExtensionConstants.kP,0,ExtensionConstants.kD,0);
    DigitalChannel limitSwitchExtension;
    Debouncer debouncerExtension;
    Encoder encoder;
    private double trueCurrentPos = 0;
    ElapsedTime timer = null;
    Boolean switchStatus = false;
    double valueTimer = 0;
    Robot robot;
    public Extension(HardwareMap hardwareMap, boolean isAuto, Robot robot) {
        this.robot  =robot;
        motor = hardwareMap.get(DcMotorEx.class, "extend");
        resetVar();
        resetMotorAndEncoder();
        limitSwitchExtension = hardwareMap.get(DigitalChannel.class, "limitSwitchExtension");
        limitSwitchExtension.setMode(DigitalChannel.Mode.INPUT);
        debouncerExtension = new Debouncer(0.3 );
    }

    public void resetVar() {
        power = 0;
        target = 0;
        currentPos = 0;
        offset = 0;
        mode = MODE.IDLE;
        angle = 0;
        maxx = 0;
        controller = new PIDFController(ExtensionConstants.kP,0,ExtensionConstants.kD,0);
        timer = null;
        valueTimer = 0;
    }

    public void resetMotorAndEncoder() {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder = new Encoder(motor);
        offset = encoder.getCurrentPosition();
        trueCurrentPos = offset;
        motor.setPower(0);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    public double raw_power = 0;
    private double currentAmp = 0;
    public boolean getSwitchStatus() {
        return switchStatus;
    }

    public boolean checkSwitch() {
        switchStatus = debouncerExtension.calculate(limitSwitchExtension.getState());
        return switchStatus;
    }
    public void setTaget(double target) {
        mode = MODE.AUTO;
        this.target = target;
        controller.reset();
    }
    public void changeRawPower(double power) {
        raw_power = power;
        mode = MODE.RAW_POWER;
    }
    double previous_angle = 0;

    private void updateFeedForward() {
        ff = ExtensionConstants.kf * Math.sin(Math.toRadians(angle));
    }
    public void pidUpdate() {
        controller.setPIDF(ExtensionConstants.kP, 0, ExtensionConstants.kD, 0);
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
    private void updateCurrentPosition() {
        trueCurrentPos = encoder.getCurrentPosition();
    }
    public double getTrueCurrentPosition() {
        return trueCurrentPos;
    }

    public double getCurrentPos(double angle) {
        return getTrueCurrentPosition() - offset;
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
    void resetEncoder() {
        offset = getTrueCurrentPosition();
    }
    public void update() {
        updateCurrentPosition();
        angle = robot.arm.pitchSubsystem.get_angle();
        currentPos = getCurrentPos(angle);
        Log.w("Debug","case +" + mode);
        Log.w("Debug","current Pos: +" + currentPos);
        Log.w("Debug","target Pos: +" + target);
        if(IS_DISABLED) return;
        if(checkSwitch()) {
            resetEncoder();
        }
        checkTimer();
        updateFeedForward();
        switch (mode) {
            case AUTO:
                raw_power = 0;
                pidUpdate();
                motor.setPower(Utils.minMaxClip(power + ff ,ExtensionConstants.minPower, ExtensionConstants.maxPower));
                break;
            case RAW_POWER:
                motor.setPower(raw_power);
                break;
            case MANUAL:
                raw_power = 0;
                motor.setPower(Utils.minMaxClip(-1,1,power + ff));
                break;
            case IDLE:
                raw_power = 0;
                if(robot.arm.targetState == ArmState.HighBasketTeleOp || robot.arm.targetState == ArmState.HIGHBASKET) {
                    motor.setPower(ff);
                }else {
                    motor.setPower(ExtensionConstants.idlePower);
                }
                break;
        }
        previous_angle = angle;
    }

}