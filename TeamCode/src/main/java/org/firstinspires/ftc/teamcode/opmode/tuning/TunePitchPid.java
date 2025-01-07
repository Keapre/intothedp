package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Line;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.Globals;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Pitch.Pitch;


/*
tune kF first
then coeficients
then the linear regression based on extension
 */

@Config
@TeleOp(name = "Tune Pitch Pid",group = "tuning")
public class TunePitchPid extends LinearOpMode {

    Robot robot;
    GamePadController gg;

    public static double target = 0;
    private double prev_target = target;
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.IS_AUTO = false;
        robot = new Robot(this,new Pose2d(0,0,0));
        robot.drive.IS_DISABLED = true;
        gg = new GamePadController(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        robot.start();
        while (opModeIsActive()) {
            gg.update();

            if(gg.aOnce()) {
                robot.arm.changePitch(target);
            }
            telemetry.addData("target",target);
            telemetry.addData("pos",robot.arm.pitchSubsystem.getCurrentPos());
            telemetry.addData("state",robot.arm.pitchSubsystem.mode);
            telemetry.addData("ff",robot.arm.pitchSubsystem.ff);
            telemetry.addData("power",robot.arm.pitchSubsystem.motor1Power);
            telemetry.addData("power 1",robot.arm.pitchSubsystem.extension1.getPower());
            telemetry.addData("power 2",robot.arm.pitchSubsystem.extension2.getPower());
            telemetry.addData("real pos",robot.arm.pitchSubsystem.extension1.getCurrentPosition());
            telemetry.addData("offset",robot.arm.pitchSubsystem.offset);
            telemetry.update();
            prev_target = target;
        }
    }
}
