package org.firstinspires.ftc.teamcode.opmode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.Globals;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;

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
        int currentTick = 0;

        waitForStart();
        robot.start();
        while(opModeIsActive()){
            gg.update();

            if(gg.aOnce()){
                ticksRecorded[currentTick] = robot.arm.pitchSubsystem.getCurrentPos();
                currentTick++;
            }

            if(gg.bOnce()){
                double tickPerDegree = ticksRecorded[0] / 15;
                for(int i = 0; i < ticksRecorded.length; i++){
                    for(int j = i+1; j < ticksRecorded.length; j++) {
                        tickPerDegree = (tickPerDegree + (ticksRecorded[j] - ticksRecorded[i]) / ((j-i)*15)) / 2;
                    }
                }
                telemetry.addData("ticks", Arrays.toString(ticksRecorded));

                telemetry.addData("Tick per degree", tickPerDegree);

            }

            telemetry.addData("ticks recorded",robot.arm.pitchSubsystem.getCurrentPos());
            telemetry.update();
        }
    }
}
