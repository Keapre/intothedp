package org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.Utils.Caching.OPColorSensor;

@Config
public class Claw {
    Servo claw;
    CachingServo leftDiffy;
    CachingServo rightDiffy;
    Arm arm;

    boolean blue;

    public static class DiffyPositions {
        public double left;
        public double right;

        public DiffyPositions(double l, double r) {
            left = l;
            right = r;
        }
    }



    public static DiffyPositions down_horizontal = new DiffyPositions(0.15, 1);
    public static DiffyPositions down_vertical = new DiffyPositions(0, 0.78);
    public static DiffyPositions mid = new DiffyPositions(0.71, 0.42);
    public static DiffyPositions up = new DiffyPositions(1, 0.13);
    public static DiffyPositions slamPos = new DiffyPositions(0.32, 0.02);
   public OPColorSensor sensor;
    //TODO:
    public static double clawOpen = 0.34;
    public static double clawClose = 0.85;
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

    public tiltMode tiltState = tiltMode.UP;
    public RotateMode rotateState = RotateMode.ORIZONTAL;
    public CLAWPOS clawPos = CLAWPOS.CLOSE;

    public Claw(HardwareMap hardwareMap, boolean isAuto,boolean blue,Arm Arm) {
        this.arm = Arm;
        this.blue = blue;
        this.claw = hardwareMap.get(Servo.class, "claw");
        this.leftDiffy = new CachingServo(hardwareMap.get(Servo.class, "diffyLeft"));
        this.rightDiffy = new CachingServo(hardwareMap.get(Servo.class, "diffyRight"));
        sensor = new OPColorSensor(hardwareMap,"intakeSensor");
    }
    public static double power = 1;
    public static double reversepower = -0.16;





    public void setTiltState(tiltMode state) {
        tiltState = state;
    }

    public tiltMode getTiltState() {
        return tiltState;
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
                    clawPos = CLAWPOS.OPEN;
                }else {
                    tiltState = tiltMode.MID;
                    rotateState = RotateMode.ORIZONTAL;
                }
            }else {
                if(sensor.isBlue()) {
                    clawPos = CLAWPOS.OPEN;
                }else {
                    rotateState = RotateMode.ORIZONTAL;
                    tiltState = tiltMode.MID;
                }
            }
        }else {
            timerClaw = null;
        }
    }
    public void update() {
//        if(arm.targetState.getPitchAngle()==0) {
//            if(lastClawPose==CLAWPOS.OPEN && clawPos == CLAWPOS.CLOSE) sensor.enableLED(true);
//            checkTook();
//        }else{
//            if(lastClawPose == CLAWPOS.CLOSE && clawPos == CLAWPOS.OPEN) sensor.enableLED(false);
//        }
        switch (clawPos) {
            case OPEN:
                claw.setPosition(clawOpen);
                break;
            case CLOSE:
                claw.setPosition(clawClose);
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