package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.Utils.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.P2Pdrive;


@Config
@Autonomous(name = "P2P Tuning", group = "Tuning")
public class P2Ptuning extends LinearOpMode {

    GamePadController gg;
    P2Pdrive dt = null;
    public static Pose2d targetPose = new Pose2d(-47,0,Math.toRadians(180));
    public static Pose2d startPose = new Pose2d(-47,-47,Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {

        gg = new GamePadController(gamepad1);
        dt = new P2Pdrive(hardwareMap,startPose,true);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dt.setTargetPose(startPose);
        waitForStart();
        while(opModeIsActive()) {
            gg.update();
            if(gg.aOnce()) {
                dt.setTargetPose(targetPose);
            }
            if(gg.bOnce()) {
                dt.setTargetPose(startPose);
            }
            dt.update();
            telemetry.addData("error on x",dt.targetPose.x - dt.currentPose.x);
            telemetry.addData("error on y",dt.targetPose.y - dt.currentPose.y);
            telemetry.addData("error on h",Math.toDegrees(dt.targetPose.heading - dt.currentPose.heading));
            telemetry.addData("Pose x", dt.currentPose.x);
            telemetry.addData("Pose y", dt.currentPose.y);
            telemetry.addData("Pose heading", Math.toDegrees(dt.currentPose.heading));
            telemetry.addData("Target x", dt.targetPose.x);
            telemetry.addData("Target y", dt.targetPose.y);
            telemetry.addData("Target heading",Math.toDegrees(dt.targetPose.heading));
            telemetry.addData("rumode",dt.driveMode);
            telemetry.addData("targetPose",dt.targetPose);
            telemetry.update();
        }
    }
}
