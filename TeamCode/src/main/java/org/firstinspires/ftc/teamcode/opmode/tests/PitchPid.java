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

import org.firstinspires.ftc.teamcode.Utils.Control.OptimizedPDController;
import org.firstinspires.ftc.teamcode.Utils.Control.PID;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.Encoder;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.WEncoder;

@Config
@TeleOp(name = "PitchPid", group = "Tests")
public class PitchPid extends LinearOpMode {

    public static double kP = 0.015, kI = 0, kD = 0.0001;
    public static double target = 0;
    public static double Kcos = 0.1;
    public static double sign = -1;
    public static double ticksPerDegree = 0;
    PID pid = new PID(kP,0, kD);
    DcMotorEx motorWithEncoder, motorWithoutEncoder;
    public static double tickesPerDegree = 9.366695;

    public static double get_angle(double measurement) {
        return (- measurement) / tickesPerDegree;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motorWithEncoder = hardwareMap.get(DcMotorEx.class, "pivot1");
        motorWithoutEncoder = hardwareMap.get(DcMotorEx.class, "pivot2");

        Encoder encoder = new Encoder(motorWithEncoder);


        double offset = motorWithEncoder.getCurrentPosition();
        while (opModeIsActive()) {
            pid.updatePID(kP,0,kD);
            double error = target -get_angle(motorWithEncoder.getCurrentPosition()-offset);
            double power = pid.update(error,-1,1);
            motorWithEncoder.setPower(sign * power);
            motorWithoutEncoder.setPower(sign * power);
            telemetry.addData("error", error);
            telemetry.addData("ff", Kcos * Math.cos(get_angle(motorWithEncoder.getCurrentPosition()-offset)));
            telemetry.addData("power", power);
            telemetry.addData("position", encoder.getCurrentPosition());
            telemetry.addData("offset",offset);
            telemetry.addData("target", target);
            telemetry.addData("kCos", Kcos);
            telemetry.addData("angle",get_angle(motorWithEncoder.getCurrentPosition()-offset));

            telemetry.update();
        }

    }


}
