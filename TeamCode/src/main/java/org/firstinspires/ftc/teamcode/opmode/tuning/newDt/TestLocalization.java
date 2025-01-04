package org.firstinspires.ftc.teamcode.opmode.tuning.newDt;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.Globals;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.TelemetryUtil;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;

@TeleOp(name = "Localization Test v2", group = "Tuning")
public class TestLocalization extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.IS_AUTO = false;
        Robot robot = new Robot(this,new Pose2d(47, 47, Math.toRadians(270)));
        GamePadController gpc = new GamePadController(gamepad1);
        robot.arm.IS_DISABLED = true;
        TelemetryUtil.setup();
        double lastLoopFinish = System.currentTimeMillis();

        waitForStart();
        robot.start();
        TelemetryUtil.setup();
        while (opModeIsActive()) {
            gpc.update();
            robot.drive.drive(gpc);

            telemetry.addData("Power Vector", String.valueOf(robot.drive.getPowerVector()));
            telemetry.addData("Sample Rate (Hz) ",1/((double)(System.currentTimeMillis() - lastLoopFinish)/1000.0));
            telemetry.addData("strafe",robot.drive.strafe);
            telemetry.addData("forward",robot.drive.forward);
            telemetry.addData("h",robot.drive.h);
            telemetry.addData("voltage",robot.getVoltage());
            lastLoopFinish = System.currentTimeMillis();
            telemetry.update();
            TelemetryUtil.sendTelemetry();
        }
    }
}
