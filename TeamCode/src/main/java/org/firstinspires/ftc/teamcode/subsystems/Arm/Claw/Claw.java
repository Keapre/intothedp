package org.firstinspires.ftc.teamcode.subsystems.Arm.Claw;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Utils.Caching.OPColorSensor;
import org.firstinspires.ftc.teamcode.Utils.Globals;


@Config
public class Claw {
    CachingServo claw;
    CachingServo leftDiffy;
    CachingServo rightDiffy;

    boolean blue;


   public OPColorSensor sensor;
    CLAWPOS lastClawPose = CLAWPOS.CLOSE;

    public enum CLAWPOS {
        OPEN,
        CLOSE

    }

    public enum RotateMode {
        VERTICAL,
        ORIZONTAL

    }

    public enum tiltMode {
        UP,
        MID,
        DOWN
    }
    Robot robot;

    public tiltMode tiltState = tiltMode.UP;
    public RotateMode rotateState = RotateMode.ORIZONTAL;
    public CLAWPOS clawPos = CLAWPOS.CLOSE;

    public Claw(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;
        this.blue = !Globals.IS_RED;
        this.claw = new CachingServo(hardwareMap.get(Servo.class, "claw"));
        this.leftDiffy = new CachingServo(hardwareMap.get(Servo.class, "diffyLeft"));
        this.rightDiffy = new CachingServo(hardwareMap.get(Servo.class, "diffyRight"));
        sensor = new OPColorSensor(hardwareMap,"intakeSensor");
        deactivateClawLed();
    }


    public void setTiltState(tiltMode state) {
        tiltState = state;
    }

    public tiltMode getTiltState() {
        return tiltState;
    }

    ElapsedTime timerClaw = null;
    public boolean checkTook() {
        if(sensor.tookit()) {
            if(timerClaw == null) {
                timerClaw = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            }
            if(blue) {
                if(sensor.isRed()) {
                    clawPos = CLAWPOS.OPEN;
                    return false;
                }else {
                    tiltState = tiltMode.MID;
                    rotateState = RotateMode.ORIZONTAL;
                    return true;
                }
            }else {
                if(sensor.isBlue()) {
                    clawPos = CLAWPOS.OPEN;
                    return false;
                }else {
                    rotateState = RotateMode.ORIZONTAL;
                    tiltState = tiltMode.MID;
                    return true;
                }
            }
        }else {
            timerClaw = null;
        }
        return false;
    }
    public boolean isUnderSample() {
        if(blue) {
            return sensor.isBlue() || sensor.isYellow();
        }else {
            return sensor.isRed() || sensor.isYellow();
        }
    }

    public void activateClawLed() {
        sensor.enableLED(true);
    }

    public void deactivateClawLed() {
        sensor.enableLED(false);
    }

    public void update() {
        switch (clawPos) {
            case OPEN:
                claw.setPosition(ClawConstants.clawOpen);
                break;
            case CLOSE:
                claw.setPosition(ClawConstants.clawClose);
                break;
        }
        lastClawPose = clawPos;
        switch (tiltState) {
            case DOWN:
                if(rotateState == RotateMode.VERTICAL){
                    leftDiffy.setPosition(ClawConstants.down_vertical.left);
                    rightDiffy.setPosition(ClawConstants.down_vertical.right);
                }
                else {
                    leftDiffy.setPosition(ClawConstants.down_horizontal.left);
                    rightDiffy.setPosition(ClawConstants.down_horizontal.right);
                }
                break;
            case MID:
                if(rotateState == RotateMode.VERTICAL) {
                    rightDiffy.setPosition(ClawConstants.slamPos.right);
                    leftDiffy.setPosition(ClawConstants.slamPos.left);

                }
                else {
                    rightDiffy.setPosition(ClawConstants.mid.right);
                    leftDiffy.setPosition(ClawConstants.mid.left);
                }
                break;
            case UP:
                rightDiffy.setPosition(ClawConstants.up.right);
                leftDiffy.setPosition(ClawConstants.up.left);
                break;

        }
    }
}