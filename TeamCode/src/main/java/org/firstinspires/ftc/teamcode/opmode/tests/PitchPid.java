package org.firstinspires.ftc.teamcode.opmode.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;


import org.firstinspires.ftc.teamcode.Utils.Utils;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.ArmVelocityCalculator;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.Encoder;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.WEncoder;

@Disabled

@Config
@TeleOp(name = "PitchPid", group = "Tests")
public class PitchPid extends LinearOpMode {

    public static double kP = 0.01, kI = 0, kD = 0.00025;
    //0.015
    public static double target = 0;
    public static double Kcos = 0;
    public static double tickPerDegree = 6.10352;
    DigitalChannel limitSwitch;
    public static double kGpowerInceput = 0.035;
    public static double kGpowerFinal = 0.18;
    PIDController pid = new PIDController(0, 0., 0);
    DcMotorEx motorWithEncoder, motorWithoutEncoder,extend;
    public static double value = 22;
    Encoder encoderExtension;

    public static double angle(double ticks) {
        return (ticks) / tickPerDegree;
    }

    public boolean atPoint(double setline,double sp) {
        return Math.abs(setline - sp) < 6;
    }
    public enum RUNMODE{
        AUTO,
        IDLE
    }
    ArmVelocityCalculator calc;
    double maxx = 0;
    public static double pre_target = 0;

    public void runOpMode() throws InterruptedException {
        waitForStart();
        calc = new ArmVelocityCalculator();
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limit");
        extend = hardwareMap.get(DcMotorEx.class,"extend");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motorWithEncoder = hardwareMap.get(DcMotorEx.class, "pivot1");
        motorWithoutEncoder = hardwareMap.get(DcMotorEx.class, "pivot2");

        double offsetExtension = -extend.getCurrentPosition();

        RUNMODE runmode = RUNMODE.IDLE;
        InterpLUT lut = new InterpLUT();

        motorWithEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Encoder encoder = new Encoder(motorWithEncoder);
        motorWithEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorWithEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorWithoutEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorWithoutEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        encoderExtension = new Encoder(extend);
        encoderExtension.setDirection(Encoder.Direction.REVERSE);
        double offset = encoder.getCurrentPosition();
        double ff = 0;
        GamePadController gg;
        gg = new GamePadController(gamepad1);
        while (opModeIsActive()) {
            gg.update();
            double Angle = angle(encoder.getCurrentPosition() - offset);
//            ff =lut.get(Utils.minMaxClip(encoderExtension.getCurrentPosition()-offsetExtension,0,699)) * Math.cos(Math.toRadians(Angle));
            ff =(kGpowerInceput + kGpowerFinal)/2 * Math.cos(Math.toRadians(Angle));

            if (limitSwitch.getState() == true) {
                ff = 0;
                Angle = 0;
                offset = encoder.getCurrentPosition();
                telemetry.addData("Button", "PRESSED");
            } else {
                telemetry.addData("Button", "NOT PRESSED");
            }
            updatePid();
            if(target == 0) pre_target = value;
            else pre_target = target;
            double error = pre_target - (encoder.getCurrentPosition() - offset);
            double power = pid.calculate((encoder.getCurrentPosition()-offset),pre_target) + ff;


            if(gg.aOnce()) {
                if(runmode == RUNMODE.AUTO) runmode = RUNMODE.IDLE;
                else runmode = RUNMODE.AUTO;
            }
            switch (runmode) {
                case IDLE:
                    if(target == 0) {
                        motorWithEncoder.setPower(0);
                        motorWithoutEncoder.setPower(0);
                    }else {
                        motorWithoutEncoder.setPower(ff);
                        motorWithEncoder.setPower(ff);
                    }
                    break;
                case AUTO:
                    if(target == 0 && atPoint(value,encoder.getCurrentPosition()-offset)){
                        runmode = RUNMODE.IDLE;
                        motorWithEncoder.setPower(0);
                        motorWithoutEncoder.setPower(0);
                    }else {
                        motorWithoutEncoder.setPower(power);
                        motorWithEncoder.setPower(power);
                    }
                    break;
            }
            telemetry.addData("runmode",runmode);
            telemetry.addData("power", power);
            telemetry.addData("position", encoder.getCurrentPosition()-offset);
            telemetry.addData("target", target);
            telemetry.addData("error", error);
            telemetry.addData("angle", Angle);
            telemetry.addData("ff",ff);
            telemetry.addData("kCos", Kcos);
            maxx = Math.max(maxx,calc.calculateVelocity(encoder.getCurrentPosition()-offset));
            telemetry.addData("v",maxx);
            telemetry.update();
        }

    }

    public void updatePid() {
        pid.setPID(kP,kI,kD);
    }
}
