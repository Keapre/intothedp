package org.firstinspires.ftc.teamcode.opmode.tuning.newDt;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.TelemetryUtil;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;

@TeleOp(name = "Localization Test v2", group = "Tuning")
public class TestLocalization extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain drive = new DriveTrain(hardwareMap,new Pose2d(47, 47, Math.toRadians(270)),false);
        GamePadController gpc = new GamePadController(gamepad1);
        TelemetryUtil.setup();

        waitForStart();
        while (opModeIsActive()) {
            gpc.update();
            drive.drive(gpc);

            telemetry.addData("Power Vector", String.valueOf(drive.getPowerVector()));
            telemetry.update();
            TelemetryUtil.sendTelemetry();
        }
    }
}
