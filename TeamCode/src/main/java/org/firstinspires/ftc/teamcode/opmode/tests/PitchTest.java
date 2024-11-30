package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Caching.OPColorSensor;
import org.firstinspires.ftc.teamcode.Utils.Utils;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.Encoder;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.Drive;

@Config
@TeleOp(name = "halloween party::)))))")
public class PitchTest extends LinearOpMode {

    DigitalChannel limitSwitch;
    Encoder encd,encoderExtension;
    DcMotorEx motorE, motor,motor1;
    OPColorSensor clSensor;
    GamePadController gg;
    public static double powerExtension = 0.95;

    public static double sign = -1;
    public double getBatteryVoltage() {
        // Read the voltage from an analog input
        double analogValue  = hardwareMap.voltageSensor.iterator().next().getVoltage();

        // Return the voltage in volts
        return analogValue;
    }
    public static double tickPerDegree = 6.10352;
    public static double kGpowerInceput = 0.06;
    public static double kGpowerFinal = 0.18;
    public static double clawClosed = 0.48;
    public static double clawOpened = 0;
    long lastChecked = System.currentTimeMillis();
    public static double threesholdTransition = 550;
    public static double clawThreeshold = 15;
    public static  double desiredVelocity = 30.0; // Degrees per second
    public static  boolean usedSensor = true;

    public static double ratioWidth = 1.0;

    public static double verticalPose = 0.14;
    public static double orizontalPose = 0.69;

    InterpLUT lut = new InterpLUT();



    enum RotateMode {
        VERTICAL,
        ORIZONTAL
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
    public static double signFeed = 1;
    public static double highTiltPos = 1;
    ServoMode modeServo = ServoMode.OPEN;
    tiltMode TiltMode = tiltMode.MID;
    Drive drive;
    PIDController pid = new PIDController(0.015,0,0.00025);
    public enum Mode {
        extended,
        idle,
        retracting
    }

    Servo claw,diffyLeft,diffyRight;

    boolean atPoint(double value,double target) {
        return Math.abs(value - target) < 5;
    }
    public double target = 0;
    RotateMode rMode = RotateMode.ORIZONTAL;

    public static double angle(double ticks) {
        return (ticks) / tickPerDegree;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limit");
        motorE = hardwareMap.get(DcMotorEx.class, "pivot1");
        diffyLeft = hardwareMap.get(Servo.class,"diffyLeft");
        claw = hardwareMap.get(Servo.class,"claw");
        diffyRight = hardwareMap.get(Servo.class,"diffyRight");
        motor = hardwareMap.get(DcMotorEx.class, "pivot2");
        motor1 = hardwareMap.get(DcMotorEx.class,"extend");
        drive = new Drive(hardwareMap,new Pose2d(0,0,0),false);

        lut.add(-50,kGpowerInceput);
        lut.add(850,kGpowerFinal);
        gg = new GamePadController(gamepad1);
        motorE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // motorE.setDirection(DcMotorSimple.Direction.REVERSE);
        motorE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        encd = new Encoder(motorE);
        encoderExtension = new Encoder(hardwareMap.get(DcMotorEx.class,"hang"));
        Mode mode = Mode.idle;
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        double ff = 0;
        lut.createLUT();
        double offset = motorE.getCurrentPosition();
        double offsetExtension = -motor1.getCurrentPosition();
        encoderExtension.setDirection(Encoder.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            gg.update();
            drive.setMotorPowersFromGamepad(gg,1.0,false,true);

            double Angle = angle(motorE.getCurrentPosition() - offset);
            if(gg.aOnce()) {
                if(modeServo == ServoMode.CLOSED){
                    modeServo = ServoMode.OPEN;
                    lastChecked = System.currentTimeMillis();
                }
                else modeServo = ServoMode.CLOSED;
            }
            if(gg.bOnce()) {
                target = 550;
            }
            if (gg.xOnce()) {
                target = 0;
            }
            if(gg.yOnce()) {
                target = 250;
            }
            if (gg.dpadRightOnce()) {
                 if(rMode == RotateMode.ORIZONTAL) {
                    rMode = RotateMode.VERTICAL;
                }
            }
            if(gg.dpadLeftOnce()) {
                if(rMode == RotateMode.VERTICAL) {
                    rMode = RotateMode.ORIZONTAL;
                }
            }
            if(gg.dpadUpOnce()) {
                if(TiltMode == tiltMode.DOWN) {
                    rMode = RotateMode.ORIZONTAL;
                    TiltMode = tiltMode.MID;
                }else if(TiltMode == tiltMode.MID) {
                    rMode = RotateMode.ORIZONTAL;
                    TiltMode = tiltMode.UP;
                }
            }else if(gg.dpadDownOnce()) {
                if(TiltMode == tiltMode.UP) {
                    rMode = RotateMode.ORIZONTAL;
                    TiltMode = tiltMode.MID;
                }else if(TiltMode == tiltMode.MID) {
                    rMode = RotateMode.ORIZONTAL;
                    TiltMode = tiltMode.DOWN;
                }
            }
            if (limitSwitch.getState() == true) {
                ff = 0;
                Angle = 0;
                offset = motorE.getCurrentPosition();
                telemetry.addData("Button", "PRESSED");
            } else {
                telemetry.addData("Button", "NOT PRESSED");
            }

            ff =signFeed*kGpowerInceput * Math.cos(Math.toRadians(Angle));
            //ff =signFeed*kGpowerFinal * Math.cos(Math.toRadians(Angle));

            if(gg.leftBumper()) {
                mode = Mode.retracting;
            }else if(gg.rightBumper()) {
                mode = Mode.extended;
            }else {
                mode = Mode.idle;
            }
            double powerSAL = gg.left_trigger  - gg.right_trigger;
            switch (modeServo){
                case CLOSED:
                    claw.setPosition(clawClosed);
                    break;
                case OPEN:
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
                    diffyRight.setPosition(0.62);
                    diffyLeft.setPosition(0.03);
                    break;
                case ORIZONTAL:
                    switch (TiltMode){
                        case DOWN:
                            diffyLeft.setPosition(0.22); //0.18
                            diffyRight.setPosition(0.81);//0.86
                            //
                            break;
                        case MID:
                            diffyRight.setPosition(0.23);
                            diffyLeft.setPosition(0.82);
                            break;
                        case UP:

                            diffyRight.setPosition(0);
                            diffyLeft.setPosition(1);
                            break;
                    }
                    break;
            }
            motor1.setPower(powerSAL);

            double power = pid.calculate((motorE.getCurrentPosition()-offset),target) + ff;

            if(atPoint(motorE.getCurrentPosition(),target)) {
                power = ff;
            }

            motorE.setPower(ff + power);
            motor.setPower(ff + power);
            telemetry.addData("mode",modeServo);
            telemetry.addData("target",target);
            telemetry.addData("use Sensor:",usedSensor);
            telemetry.addData("threeshold:",threesholdTransition);
            telemetry.addData("rotate mode",rMode);
            telemetry.addData("tilt",TiltMode);
            telemetry.addData("Wrapperencoder extension without ",encoderExtension.getCurrentPosition());
            telemetry.addData("Wrapperencoder extension",encoderExtension.getCurrentPosition()-offsetExtension);
            telemetry.addData("encoder extension",(-motor1.getCurrentPosition())-offsetExtension);
            telemetry.addData("offset extension",offsetExtension);
            telemetry.addData("power",power);
            telemetry.addData("ff",ff);
            telemetry.addData("angle",Angle);
            telemetry.addData("wrapper Position",encd.getCurrentPosition());
            telemetry.addData("position", motorE.getCurrentPosition());
            telemetry.addData("hz",1/((double)(System.currentTimeMillis() - lastChecked)/1000.0));
            telemetry.addData("mode extension",mode);
            lastChecked = System.currentTimeMillis();
            telemetry.update();
        }
    }
}
