package org.firstinspires.ftc.teamcode.opmode.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Utils.Wrappers.Encoder;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.WEncoder;

@Config
@TeleOp(name = "PitchPid", group = "Tests")
public class PitchPid extends LinearOpMode {

    public static double kP = 0, kI = 0, kD = 0;
    public static double target = 0;
    public static double Kcos = 0;
    public static double ticksPerDegree = 0;
    PIDController pid = new PIDController(0, 0., 0);
    DcMotorEx motorWithEncoder, motorWithoutEncoder;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motorWithEncoder = hardwareMap.get(DcMotorEx.class, "pivot1");
        motorWithoutEncoder = hardwareMap.get(DcMotorEx.class, "pivot2");

        WEncoder encoder = new WEncoder(new Encoder(motorWithEncoder));
        pid.setTolerance(1);

        motorWithEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorWithEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorWithoutEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorWithoutEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double offset = encoder.getCurrentPosition();
        while (opModeIsActive()) {
            updatePid();
            double error = target - (encoder.getCurrentPosition() - offset);
            double power = pid.calculate(error) + Kcos * Math.cos((encoder.getCurrentPosition() - offset)/ticksPerDegree);
            motorWithEncoder.setPower(power);
            motorWithoutEncoder.setPower(power);
            telemetry.addData("error", error);
            telemetry.addData("power", power);
            telemetry.addData("position", encoder.getCurrentPosition());
            telemetry.addData("target", target);
            telemetry.addData("kCos", Kcos);

            telemetry.update();
        }

    }

    public void updatePid() {
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
    }
}
