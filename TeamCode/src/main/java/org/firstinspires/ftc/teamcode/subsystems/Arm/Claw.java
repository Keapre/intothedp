package org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.opmode.tests.PitchTest;

@Config
public class Claw {
    CachingServo claw;
    CachingServo leftDiffy;
    CachingServo rightDiffy;

    public boolean clawEnabled = true;
    public boolean rotateEnabled = true;
    public boolean tiltEnabled = true;

    public static class DiffyPositions {
        public double left = 0.0;
        public double right = 0.0;

        public DiffyPositions(double l, double r) {
            left = l;
            right = r;
        }
    }

    ;

    public static DiffyPositions down_horizontal = new DiffyPositions(0.17, 0.97);
    public static DiffyPositions down_vertical = new DiffyPositions(0, 0.77);
    public static DiffyPositions mid = new DiffyPositions(0.71, 0.42);
    public static DiffyPositions up = new DiffyPositions(1, 0.13);
    public static DiffyPositions slamPos = new DiffyPositions(0.32, 0.02);
    //TODO:
    public static double clawOpen = 0.3;
    public static double clawClose = 0.82;

    public enum CLAWPOS {
        OPEN,
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

    public Claw(HardwareMap hardwareMap, boolean isAuto) {
        this.claw = new CachingServo(hardwareMap.get(Servo.class, "claw"));
        this.leftDiffy = new CachingServo(hardwareMap.get(Servo.class, "diffyLeft"));
        this.rightDiffy = new CachingServo(hardwareMap.get(Servo.class, "diffyRight"));
    }


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

    public void update() {
        switch (clawPos) {
            case OPEN:
                claw.setPosition(clawOpen);
                break;
            case CLOSE:
                claw.setPosition(clawClose);
                break;
        }
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