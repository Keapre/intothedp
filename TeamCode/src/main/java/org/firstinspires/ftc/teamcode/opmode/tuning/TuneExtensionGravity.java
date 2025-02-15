package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "Tune Extension Gravity",group = "tuning")
public class TuneExtensionGravity extends OpMode {
    Robot robot = null;
    public static double multiplier = 1;

    @Override
    public void start() {
        robot.start();
    }
    @Override
    public void init() {
        robot = new Robot(this,new Pose2d(0,0,0));
        robot.arm.extensionSubsystem.IS_DISABLED = true;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.arm.pitchSubsystem.setTUNING_FF(true);
    }

    @Override
    public void stop() {
        robot.stop();
    }
    @Override
    public void loop() {
        robot.arm.pitchSubsystem.setMultiplier(multiplier);

        telemetry.addData("current Pos Pivot",robot.arm.pitchSubsystem.getCurrentPos());
        telemetry.addData("current Angle Pivot",robot.arm.pitchSubsystem.angle);

        telemetry.addData("current Pos Extension",robot.arm.extensionSubsystem.currentPos);

        telemetry.addData("multiplier",multiplier);
        telemetry.addData("ff pivot",robot.arm.pitchSubsystem.ff);

        telemetry.update();
    }
}
