package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Utils.Wrappers.Encoder;

@TeleOp(name = "ExtendPos")
public class ExtendPos extends LinearOpMode {
    DcMotorEx extendMotor;
    DcMotorEx port;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        extendMotor = hardwareMap.get(DcMotorEx.class,"extend");
        Encoder encoder = new Encoder(extendMotor);
        while (opModeIsActive()) {
            extendMotor.setPower(gamepad1.left_stick_y);
            telemetry.addData("Extend Position", extendMotor.getCurrentPosition());
            telemetry.addData("extend position current",encoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
