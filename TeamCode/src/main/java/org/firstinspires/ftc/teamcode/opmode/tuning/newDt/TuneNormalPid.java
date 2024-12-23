package org.firstinspires.ftc.teamcode.opmode.tuning.newDt;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utils.Wrappers.TelemetryUtil;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;

@Config
@Autonomous(name = "Tune Normal PID", group = "Tuning")
public class TuneNormalPid extends LinearOpMode {

    public static boolean useSecondPid = false;
    public static double x = 0,y = 0.0,z = 0;
    @Override
    public void runOpMode() throws InterruptedException {


        DriveTrain dt = new DriveTrain(hardwareMap,new Pose2d(47, 47, Math.toRadians(270)),false);
        TelemetryUtil.setup();
        waitForStart();
        while (opModeIsActive()) {
            if(dt.state!= DriveTrain.STATE.IDLE) dt.goToPoint(new Pose2d(x,y,z),useSecondPid,true,0.85);

            telemetry.addData("state",dt.state);
            telemetry.addData("x",x);
            telemetry.addData("y",y);
            telemetry.addData("z",z);
            telemetry.update();
            TelemetryUtil.sendTelemetry();
        }
    }
}
