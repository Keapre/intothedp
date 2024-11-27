package org.firstinspires.ftc.teamcode.opmode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "ExtendPos")
public class ExtendPos extends LinearOpMode {
    DcMotorEx extendMotor;
    DcMotorEx port;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        extendMotor = hardwareMap.get(DcMotorEx.class,"extend");
        while (opModeIsActive()) {
            extendMotor.setPower(gamepad1.left_stick_y);
            telemetry.addData("Extend Position", extendMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
