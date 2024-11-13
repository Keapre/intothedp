package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Utils.Caching.OPColorSensor;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.Encoder;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.Drive;

@Config
@TeleOp(name = "halloween party::)))))")
public class PitchTest extends LinearOpMode {

    DigitalChannel limitSwitch;
    Encoder encd;
    DcMotorEx motorE, motor,motor1;
    OPColorSensor clSensor;
    GamePadController gg;
    public static double powerExtension = 0.9;

    public static double sign = -1;
    public double getBatteryVoltage() {
        // Read the voltage from an analog input
        double analogValue  = hardwareMap.voltageSensor.iterator().next().getVoltage();

        // Return the voltage in volts
        return analogValue;
    }
    public static double tickPerDegree = 9.366695;
    public static double kGpowerInceput = 0.037;
    public static double kGpowerFinal = 0.15;
    public static double clawClosed = 0.45;
    public static double clawOpened = 0.2;
    long lastChecked = System.currentTimeMillis();
    public static double threesholdTransition = 550;
    public static double clawThreeshold = 15;
    public static  double desiredVelocity = 30.0; // Degrees per second
    public static  boolean usedSensor = true;

    public static double ratioWidth = 1.0;

    public static double verticalPose = 0.14;
    public static double orizontalPose = 0.69;



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
    public static double signFeed = -1;
    public static double highTiltPos = 1;
    tiltMode TiltMode = tiltMode.MID;
    Drive drive;
    public enum Mode {
        extended,
        idle,
        retracting
    }

    public static double angle(double ticks) {
        return (-ticks) / tickPerDegree;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limit");
        motorE = hardwareMap.get(DcMotorEx.class, "pivot1");
        motor = hardwareMap.get(DcMotorEx.class, "pivot2");
        motor1 = hardwareMap.get(DcMotorEx.class,"hang");
        drive = new Drive(hardwareMap,new Pose2d(0,0,0),false);
        encd = new Encoder(motorE);
        gg = new GamePadController(gamepad1);
       // motorE.setDirection(DcMotorSimple.Direction.REVERSE);
        motorE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Mode mode = Mode.idle;
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        double ff = 0;

        double offset = motorE.getCurrentPosition();
        waitForStart();
        while (opModeIsActive()) {
            gg.update();
            drive.setMotorPowersFromGamepad(gg,1.0,false,true);
            double power = -gg.left_trigger + gg.right_trigger;
            double Angle = angle(motorE.getCurrentPosition() - offset);
            ff =signFeed*kGpowerInceput * Math.cos(Math.toRadians(Angle));
            if(gg.leftBumper()) {
                mode = Mode.retracting;
            }else if(gg.rightBumper()) {
                mode = Mode.extended;
            }else {
                mode = Mode.idle;
            }
            switch (mode) {
                case idle:
                    motor1.setPower(0);
                    break;
                case extended:
                    motor1.setPower(sign * powerExtension);
                    break;
                case retracting:
                    motor1.setPower(sign * -1 * powerExtension);
                    break;
            }
            if (limitSwitch.getState() == true) {
                telemetry.addData("Button", "PRESSED");
            } else {
                telemetry.addData("Button", "NOT PRESSED");
            }
            power*=12.0/getBatteryVoltage();
            motorE.setPower(ff + power);
            motor.setPower(ff + power);
            telemetry.addData("power",power);
            telemetry.addData("ff",ff);
            telemetry.addData("angle",Angle);
            telemetry.addData("wrapper Position",encd.getCurrentPosition());
            telemetry.addData("position", motorE.getCurrentPosition());
            telemetry.addData("hz",(System.currentTimeMillis()-lastChecked)/1000);
            lastChecked = System.currentTimeMillis();
            telemetry.update();
        }
    }
}
