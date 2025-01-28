package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "pid tuner extension 2")
public class PidTuner extends LinearOpMode {

    public static double kP = 0.03,kD =0.0005,kF = 0.07; // sin
    public static double target = 0;
    DcMotorEx motor;
    PIDFController pidf = new PIDFController(kP,0,kD,0);

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class,"extend");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            pidf.setPIDF(kP,0,kD,0);
            double currentPos = motor.getCurrentPosition();
            double power = pidf.calculate(currentPos,target);
            motor.setPower(power);
            telemetry.addData("currentPos",currentPos);
            telemetry.addData("error",target-currentPos);
            telemetry.addData("power",power);
            telemetry.addData("target",target);
            telemetry.update();
        }
    }
}
