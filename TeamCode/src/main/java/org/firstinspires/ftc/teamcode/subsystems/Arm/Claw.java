package org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.Caching.CachingCRServo;
import org.firstinspires.ftc.teamcode.Utils.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Utils.Caching.OPColorSensor;
import org.firstinspires.ftc.teamcode.opmode.tests.PitchTest;

@Config
public class Claw {
    CachingCRServo claw;
    CachingServo leftDiffy;
    CachingServo rightDiffy;

    public boolean clawEnabled = true;
    public boolean rotateEnabled = true;
    public boolean tiltEnabled = true;
    boolean blue =false;

    public static class DiffyPositions {
        public double left = 0.0;
        public double right = 0.0;

        public DiffyPositions(double l, double r) {
            left = l;
            right = r;
        }
    }

    ;

    public static DiffyPositions down_horizontal = new DiffyPositions(0.15, 1);
    public static DiffyPositions down_vertical = new DiffyPositions(0, 0.77);
    public static DiffyPositions mid = new DiffyPositions(0.71, 0.42);
    public static DiffyPositions up = new DiffyPositions(1, 0.13);
    public static DiffyPositions slamPos = new DiffyPositions(0.32, 0.02);
   public OPColorSensor sensor;
    //TODO:
    public static double clawOpen = 0.3;
    public static double clawClose = 0.82;
    CLAWPOS lastClawPose = CLAWPOS.CLOSE;

    public enum CLAWPOS {
        FORWARD,
        REVERSE,
        CLOSE

    }

    public enum RotateMode {
        VERTICAL,
        ORIZONTAL;

    }

    public enum tiltMode {
        UP,
        MID,
        DOWN
    }

    public tiltMode tiltState = tiltMode.UP;
    public RotateMode rotateState = RotateMode.ORIZONTAL;
    public CLAWPOS clawPos = CLAWPOS.CLOSE;

    public Claw(HardwareMap hardwareMap, boolean isAuto,boolean blue) {
        this.blue = blue;
        this.claw = new CachingCRServo(hardwareMap.get(CRServo.class, "test"));
        this.leftDiffy = new CachingServo(hardwareMap.get(Servo.class, "diffyLeft"));
        this.rightDiffy = new CachingServo(hardwareMap.get(Servo.class, "diffyRight"));
        sensor = new OPColorSensor(hardwareMap,"intakeSensor");
    }
    public static double power = 1;
    public static double reversepower = -0.16;


    public void setClawPos(CLAWPOS pos) {
        clawPos = pos;
    }

    public void setRotateState(RotateMode state) {
        rotateState = state;
    }

    public void setTiltState(tiltMode state) {
        tiltState = state;
    }

    public tiltMode getTiltState() {
        return tiltState;
    }

    public RotateMode getRotateState() {
        return rotateState;
    }

    public CLAWPOS getClawPos() {
        return clawPos;
    }

    public void  setReverse(double speed) {
        reversepower = speed;
    }
    ElapsedTime timerClaw = null;
    void checkTook() {
        if(sensor.tookit()) {
            if(timerClaw == null) {
                timerClaw = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            }
            if(blue) {
                if(sensor.isRed()) {
                    clawPos = CLAWPOS.REVERSE;
                }else {
                    if(timerClaw.time() > 1500) {
                        clawPos = CLAWPOS.CLOSE;
                    }
                    tiltState = tiltMode.MID;
                    rotateState = RotateMode.ORIZONTAL;
                }
            }else {
                if(sensor.isBlue()) {
                    clawPos = CLAWPOS.REVERSE;
                }else {
                    if(timerClaw.time() > 1500) {
                        clawPos = CLAWPOS.CLOSE;
                    }
                    rotateState = RotateMode.ORIZONTAL;
                    tiltState = tiltMode.MID;
                }
            }
        }else {
            timerClaw = null;
        }
    }
    public void update() {
        if(clawPos == CLAWPOS.FORWARD) {
            if(lastClawPose!=CLAWPOS.FORWARD) sensor.enableLED(true);
            checkTook();
        }else{
            if(lastClawPose == CLAWPOS.FORWARD) sensor.enableLED(false);
        }
        switch (clawPos) {
            case FORWARD:
                claw.setPower(power);
                break;
            case CLOSE:
                claw.setPower(0);
                break;
            case REVERSE:
                claw.setPower(reversepower);
                break;
        }
        lastClawPose = clawPos;
        switch (tiltState) {
            case DOWN:
                if(rotateState == RotateMode.VERTICAL){
                    leftDiffy.setPosition(down_vertical.left);
                    rightDiffy.setPosition(down_vertical.right);
                }
                else {
                    leftDiffy.setPosition(down_horizontal.left);
                    rightDiffy.setPosition(down_horizontal.right);
                }
                break;
            case MID:
                if(rotateState == RotateMode.VERTICAL) {
                    rightDiffy.setPosition(slamPos.right);
                    leftDiffy.setPosition(slamPos.left);
                }
                else {
                    rightDiffy.setPosition(mid.right);
                    leftDiffy.setPosition(mid.left);
                }
                break;
            case UP:
                rightDiffy.setPosition(up.right);
                leftDiffy.setPosition(up.left);
                break;

        }
    }
}