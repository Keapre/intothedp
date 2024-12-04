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
        extendMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Encoder encoder = new Encoder(extendMotor);
        extendMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        double offset = encoder.getCurrentPosition();
        while (opModeIsActive()) {
            extendMotor.setPower(gamepad1.left_stick_y);
            telemetry.addData("Extend Position", extendMotor.getCurrentPosition() - offset);
            telemetry.addData("extend position current",encoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
