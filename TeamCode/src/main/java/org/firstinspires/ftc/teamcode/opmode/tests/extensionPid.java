package org.firstinspires.ftc.teamcode.opmode.tests;

import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Utils.Utils;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.Encoder;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.WEncoder;

@Config
@TeleOp(name = "extensionPid", group = "Tests")
public class extensionPid extends LinearOpMode {

    public static double kP = 0.002, kI = 0, kD = 0;
    PIDController pid = new PIDController(0, 0, 0);
    public static double kG = 10.18;
    public static double target = 0;

    public static double sign = -   1;
    DcMotorEx extension;

    public static double angle = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        extension = hardwareMap.get(DcMotorEx.class, "extend");
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Encoder encoder = new Encoder(extension);
        double offset = encoder.getCurrentPosition();

        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            updatePID();

            double pos = (encoder.getCurrentPosition() - offset);
            double error = target - (encoder.getCurrentPosition()-offset);
            double power=pid.calculate(pos,target);
            power*=sign;
            extension.setPower(Utils.minMaxClip(power, -0.5, 0.5    ));
            telemetry.addData("error", error);
            telemetry.addData("power", power);
            telemetry.addData("curentPos", encoder.getCurrentPosition());
            telemetry.addData("position", encoder.getCurrentPosition()-offset);
            telemetry.addData("angle", angle);
            telemetry.addData("target", target);
            telemetry.addData("kg", kG);
            telemetry.update();
        }
    }
    public void updatePID() {
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
    }
}
