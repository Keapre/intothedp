package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.Drive;


@TeleOp(name = "drivetraintest")
public class DriveTrainTest extends LinearOpMode {

    Drive drive;
    GamePadController gg;
    long lastLoopFinish;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drive(hardwareMap,new Pose2d(0,0,0),false);
        gg = new GamePadController(gamepad1);
        lastLoopFinish = System.currentTimeMillis();
        waitForStart();
        while(opModeIsActive()) {
            gg.update();
            drive.setMotorPowersFromGamepad(gg,0.8,false,true);
            telemetry.addData("slow_mode",drive.slow_mode);
            telemetry.addData("Sample Rate (Hz) ",1/((double)(System.currentTimeMillis() - lastLoopFinish)/1000.0));
            lastLoopFinish = System.currentTimeMillis();
            telemetry.update();
        }
    }
}
