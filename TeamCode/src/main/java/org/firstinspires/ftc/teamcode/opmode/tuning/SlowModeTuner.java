package org.firstinspires.ftc.teamcode.opmode.tuning;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.Globals;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;

@TeleOp(name = "slow mode tuner", group = "Tuning")
public class SlowModeTuner extends LinearOpMode {

    Robot robot = null;
    GamePadController gg;
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.IS_AUTO = false;
        robot = new Robot(this,new Pose2d(0,0,Math.toRadians(270)));
        gg = new GamePadController(gamepad1);

        waitForStart();
        robot.start();
        while (opModeIsActive()) {

            if(gg.aOnce()) {
                robot.drive.slow_mode = !robot.drive.slow_mode;
            }

            if(gg.dpadUpOnce()) {
                robot.drive.xSlowAdjust(0.05);
            }
            if(gg.dpadDownOnce()) {
                robot.drive.xSlowAdjust(-0.05);
            }

            if(gg.dpadRightOnce()) {
                robot.drive.ySlowAdjust(0.05);
            }

            if(gg.dpadLeftOnce()) {
                robot.drive.ySlowAdjust(-0.05);
            }

            if(gg.rightBumperOnce()) {
                robot.drive.hSlowAdjust(0.05);
            }
            if(gg.leftBumperOnce()) {
                robot.drive.hSlowAdjust(-0.05);
            }
            robot.drive.drive(gg);

            telemetry.addData("x slow",robot.drive.xSlowModeMultipler);
            telemetry.addData("y slow",robot.drive.ySlowModeMultiplier);
            telemetry.addData("h slow",robot.drive.hSlowModeMultiplier);
            telemetry.addData("slow mode",robot.drive.slow_mode);

            telemetry.update();
        }
    }
}
