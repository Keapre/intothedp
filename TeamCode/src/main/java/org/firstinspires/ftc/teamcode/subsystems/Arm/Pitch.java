package org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Caching.CachingDcMotorEx;
import org.firstinspires.ftc.teamcode.Utils.Utils;

@Config
public class Pitch {

    public enum PITCHPOS {
        UP(0),
        HIGH_BASKET(0),
        LOW_BASKET(0),

        HIGH_CHAMBER(0),
        LOW_CHAMBER(0),

        DOWN(0);

        public double position;
        PITCHPOS(double x) {
            position = x;
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

        public static double tickperDegree = 0.0;

        public static double Kcos = 0.0;
    }

    //Params PARAMS = new Params();
    public double motor1Power = 0;
    public double motor2Power = 0;
    CachingDcMotorEx extension1; // with encoder
    CachingDcMotorEx extension2; // non ecoder

    public PIDFController controller = new PIDFController(Params.kP, Params.kI, Params.kD,0);

    public static double ff = 0;
    public static double angle = 0;
    public static double offset = 0;
    public static double currentPos = 0;
    public MODE mode = MODE.AUTO;
    public PITCHPOS target = PITCHPOS.DOWN;

    public double tickperDegree = 9.366695;

    public Pitch(HardwareMap hardwareMap,boolean isAutonomous) {
        extension2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"pivot2"),0.05);
        extension1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class,"pivot1"),0.05);

        initializeMotors();

        if(isAutonomous) {
            mode = MODE.AUTO;
        }
        currentPos = extension1.getCurrentPosition();
        offset = currentPos;
    }

    public void initializeMotors() {
        extension2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extension1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        extension1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extension2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //REVERSE IF NEEDED
    }

    public boolean atTarget() {
        if(mode == MODE.AUTO) {
            if(controller.atSetPoint()) return true;
            else return false;
        }
        return false;
    }
    public boolean isAtPosition(PITCHPOS position) {
        double targetCounts = calculateAngle() *tickperDegree ;
        double currentPosition = extension1.getCurrentPosition();
        return Math.abs(currentPosition - targetCounts) < (2 * tickperDegree); // Within 2 degrees
    }

    public void setTarget(PITCHPOS pos) {
        target = pos;
    }
    public double calculateAngle() {
        double tempDegree = currentPos / Params.tickperDegree;
        return tempDegree;
    }

    public void setPowerMotors(double power) {
        this.motor1Power = power;
        this.motor2Power = power;
    }

    public void pid() {
        controller.setSetPoint(target.position);
        if(controller.atSetPoint()) {
            mode = MODE.IDLE;
            return;
        }
        mode = MODE.AUTO;
        controller.setPIDF(Params.kP, Params.kI, Params.kD,ff);

        motor1Power = controller.calculate(currentPos);
        motor2Power = motor1Power;


    }

    public void update() {
        currentPos = extension1.getCurrentPosition() - offset;
        angle = calculateAngle();
        ff = Math.cos(angle) * Params.Kcos;
        switch (mode) {
            case AUTO:
                pid();
                extension1.setPower(Utils.minMaxClip(-1,1,motor1Power ));
                extension2.setPower(Utils.minMaxClip(-1,1,motor2Power));
                break;
            case MANUAL:
                extension1.setPower(Utils.minMaxClip(-1,1,motor1Power + ff));
                extension2.setPower(Utils.minMaxClip(-1,1,motor2Power + ff));
                break;
            case IDLE:
                extension1.setPower(ff);
                extension2.setPower(ff);
                break;
        }
    }


}
