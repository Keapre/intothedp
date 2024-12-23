package org.firstinspires.ftc.teamcode.opmode.tuning.newDt;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.Wrappers.TelemetryUtil;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;

@Config
@TeleOp(name = "Tune Ks", group = "Tuning")
public class TuneKs extends LinearOpMode {

    public static double x = 0,y = 0.01,z = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryUtil.setup();
        DriveTrain drive = new DriveTrain(hardwareMap,new Pose2d(47, 47, Math.toRadians(270)),false);
        waitForStart();
        while (opModeIsActive()) {
            drive.setCustomPowerVector(new Pose2d(new Vector2d(x,y),z));
            TelemetryUtil.sendTelemetry();
        }
    }
}
