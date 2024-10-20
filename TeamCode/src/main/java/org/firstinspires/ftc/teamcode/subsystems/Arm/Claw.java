package org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Caching.CachingServo;

@Config
public class Claw {
    CachingServo claw;
    CachingServo rotate;
    CachingServo tilt;

    public boolean clawEnabled = true;
    public boolean rotateEnabled = true;
    public boolean tiltEnabled = true;

    //TODO:
    public static double clawOpen = 0.0;
    public static double clawClose = 0.0;

    public static double left90 = 0.0;
    public static double left45 = 0.0;
    public static double defaultRotate = 0.0;
    public static double right45 = 0.0;
    public static double right90 = 0.0;


    public static double tiltUp = 0.0;
    public static double tiltDown = 0.0;
    public static double tiltMid = 0.0;
    public enum CLAWPOS {
        OPEN(clawOpen),
        CLOSE(clawClose);

        public double position;
         CLAWPOS(double x) {
            position = x;
        }
    }

    public enum ROTATESTATE {
        //previous, current, next

        LEFT90(left90){
            @Override
            public ROTATESTATE previous() {
                return this;
            }
        },
        LEFT45(left45),
        DEFAULT(defaultRotate),
        RIGHT45(right45),


        RIGHT90(right90){
            @Override
            public ROTATESTATE next() {
                return this;
            }
        };

        public final double pos;

        ROTATESTATE(double pos) {
            this.pos = pos;
        }

        public ROTATESTATE previous() {
            return values()[ordinal() - 1];
        }

        public ROTATESTATE next() {
            return values()[ordinal() + 1];
        }
    }


    public enum TILTSTATE {
        UP(tiltUp),
        MID(tiltMid),
        DOWN(tiltDown);

        public final double pos;

        TILTSTATE(double pos) {
            this.pos = pos;
        }
    }

    public TILTSTATE tiltState = TILTSTATE.DOWN;
    public ROTATESTATE rotateState = ROTATESTATE.DEFAULT;
    public CLAWPOS clawPos = CLAWPOS.OPEN;

    public Claw(HardwareMap hardwareMap) {
        this.claw = new CachingServo(hardwareMap.get(Servo.class, "claw"));
        this.rotate = new CachingServo(hardwareMap.get(Servo.class, "rotate"));
        this.tilt = new CachingServo(hardwareMap.get(Servo.class, "tilt"));
    }


    public void setClawPos(CLAWPOS pos) {
        clawPos = pos;
    }

    public void setRotateState(ROTATESTATE state) {
        rotateState = state;
    }

    public void setTiltState(TILTSTATE state) {
        tiltState = state;
    }

    public TILTSTATE getTiltState(){
        return tiltState;
    }

    public ROTATESTATE getRotateState(){
        return rotateState;
    }

    public CLAWPOS getClawPos(){
        return clawPos;
    }

    public void update() {
        if(clawEnabled) {
            switch (clawPos) {
                case OPEN:
                    claw.setPosition(CLAWPOS.OPEN.position);
                    break;
                case CLOSE:
                    claw.setPosition(CLAWPOS.CLOSE.position);
                    break;
                default:
                    break;
            }
        }
        if(rotateEnabled) {
            switch (rotateState) {
                case LEFT90:
                    rotate.setPosition(ROTATESTATE.LEFT90.pos);
                    break;
                case LEFT45:
                    rotate.setPosition(ROTATESTATE.LEFT45.pos);
                    break;
                case DEFAULT:
                    rotate.setPosition(ROTATESTATE.DEFAULT.pos);
                    break;
                case RIGHT45:
                    rotate.setPosition(ROTATESTATE.RIGHT45.pos);
                    break;
                case RIGHT90:
                    rotate.setPosition(ROTATESTATE.RIGHT90.pos);
                    break;
                default:
                    break;
            }
        }

        if(tiltEnabled) {
            switch (tiltState) {
                case UP:
                    tilt.setPosition(TILTSTATE.UP.pos);
                    break;
                case MID:
                    tilt.setPosition(TILTSTATE.MID.pos);
                    break;
                case DOWN:
                    tilt.setPosition(TILTSTATE.DOWN.pos);
                    break;
                default:
                    break;
            }
        }
    }
}
