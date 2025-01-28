package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.Globals;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Pitch.Pitch;

import java.lang.reflect.Array;
import java.util.Arrays;


@TeleOp
public class FindTickPerDegree extends LinearOpMode {

    GamePadController gg;
    Robot robot = null;
    Double[] ticksRecorded = new Double[180/15];
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.IS_AUTO = false;
        robot = new Robot(this, new Pose2d(0,0,0));
        gg = new GamePadController(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int currentTick = 0;

        waitForStart();
        robot.start();
        while(opModeIsActive()){
            robot.arm.pitchSubsystem.mode = Pitch.MODE.IDLE;
            telemetry.addData("ticks recorded",robot.arm.pitchSubsystem.getCurrentPos());
            telemetry.update();
        }
    }
}
