package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Caching.OPColorSensor;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;

import java.util.List;

@Config
@TeleOp(name = "MiscociOuttake")
public class ArmPreTest extends LinearOpMode {

    Servo claw,rotate,tilt;
    OPColorSensor clSensor;

    GamePadController gamepadd;
    double width = 0,length = 0;

    void getDimensions(LLResultTypes.DetectorResult detector) {
        List<List<Double>> points = detector.getTargetCorners();
        width = Math.abs(points.get(0).get(0) - points.get(1).get(0)) + 1;
        length = Math.abs(points.get(0).get(1) - points.get(1).get(1)) + 1;
    }

    public static double clawClosed = 0.4;
    public static double clawOpened = 0.2;
    long lastChecked = System.currentTimeMillis();
    public static double threesholdTransition = 550;
    public static double clawThreeshold = 15;
    public static  boolean usedSensor = true;

    public static double ratioWidth = 1.0;

    public static double verticalPose = 0.17;
    public static double orizontalPose = 0.73;



    enum RotateMode {
        VERTICAL(verticalPose),
        ORIZONTAL(orizontalPose);

        double x;

         RotateMode(double x) {
            this.x = x;
        }
    }



    enum ServoMode{
        CLOSED,
        PRE_CLOSED,
        OPEN
    }

    enum tiltMode {
        UP(highTiltPos){
            @Override
            public tiltMode next() {
                return this;
            }
        },
        MID(tiltMidPos),
        DOWN(lowTiltPos) {
            @Override
            public tiltMode previous() {
                return this;
            }
        };

        public final double pos;
        tiltMode(double pos) {
            this.pos = pos;
        }
        public tiltMode previous() {
            return values()[ordinal()-1];
        }
        public tiltMode next() {
            return values()[ordinal()+1];
        }
    }

    public static double tiltMidPos = 0.5;
    public static double lowTiltPos = 0;
    public static double highTiltPos = 1;
    public long lastLoopFinish;
    tiltMode TiltMode = tiltMode.MID;
    @Override
    public void runOpMode() throws InterruptedException {
        ServoMode modeServo = ServoMode.OPEN;

        lastLoopFinish = System.currentTimeMillis();
        tilt = hardwareMap.get(Servo.class,"tilt");
        claw = hardwareMap.get(Servo.class,"claw");
        rotate = hardwareMap.get(Servo.class,"rotate");
            RotateMode rMode = RotateMode.ORIZONTAL;
        clSensor =new OPColorSensor(hardwareMap.get(ColorRangeSensor.class,"sensor"));
        gamepadd = new GamePadController(gamepad1);
        waitForStart();

        while(opModeIsActive()) {
            clSensor.update();
            gamepadd.update();
            if(gamepadd.aOnce()) {
                if(modeServo == ServoMode.CLOSED){
                    modeServo = ServoMode.OPEN;
                    lastChecked = System.currentTimeMillis();
                }
                else modeServo = ServoMode.CLOSED;
            }
            if(gamepadd.bOnce()) {
                usedSensor = !usedSensor;
            }
            if (gamepadd.xOnce()) {
                if(rMode == RotateMode.VERTICAL) {
                    rMode = RotateMode.ORIZONTAL;
                }else if(rMode == RotateMode.ORIZONTAL) {
                    rMode = RotateMode.VERTICAL;
                }
            }
            if(gamepadd.dpadUpOnce()) {
                if(TiltMode == tiltMode.DOWN) {
                    TiltMode = tiltMode.MID;
                }else if(TiltMode == tiltMode.MID) {
                    TiltMode = tiltMode.UP;
                }
            }else if(gamepadd.dpadDownOnce()) {
                if(TiltMode == tiltMode.UP) {
                    TiltMode = tiltMode.MID;
                }else if(TiltMode == tiltMode.MID) {
                    TiltMode = tiltMode.DOWN;
                }
            }

            switch (modeServo){
                case CLOSED:
                    claw.setPosition(clawClosed);
                    break;
                case OPEN:
//                    if(usedSensor && clSensor.getDistance() < clawThreeshold && System.currentTimeMillis() - lastChecked > threesholdTransition) {
//                        lastChecked = System.currentTimeMillis();
//                        modeServo = ServoMode.CLOSED;
//                        break;
//                    }
                    claw.setPosition(clawOpened);
                    break;
                case PRE_CLOSED:
                    if(System.currentTimeMillis() - lastChecked >= threesholdTransition) {
                        modeServo = ServoMode.CLOSED;
                    }
                    break;
                default:
                    break;
            }
            switch (rMode){
                case VERTICAL:
                    rotate.setPosition(verticalPose);
                    break;
                case ORIZONTAL:
                    rotate.setPosition(orizontalPose);
                    break;

            }

            switch (TiltMode) {
                case DOWN:
                    tilt.setPosition(lowTiltPos);
                    break;
                case MID:
                    tilt.setPosition(tiltMidPos);
                    break;
                case UP:
                    tilt.setPosition(highTiltPos);
                    break;
            }
            telemetry.addData("distance",clSensor.getDistance());
            telemetry.addData("mode",modeServo);
            telemetry.addData("use Sensor:",usedSensor);
            telemetry.addData("threeshold:",threesholdTransition);
            telemetry.addData("rotate mode",rMode);
            telemetry.addData("tilt",TiltMode);
            telemetry.addData("Sample Rate (Hz) ",1/((double)(System.currentTimeMillis() - lastLoopFinish)/1000.0));
            lastLoopFinish = System.currentTimeMillis();
            telemetry.update();
        }

    }
}
