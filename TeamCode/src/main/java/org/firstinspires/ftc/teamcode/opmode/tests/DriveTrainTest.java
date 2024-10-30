package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.MecanumDrive;

@TeleOp(name = "drivetraintest")
public class DriveTrainTest extends LinearOpMode {

    MecanumDrive drive;
    GamePadController gg;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0),false);
        gg = new GamePadController(gamepad1);
        waitForStart();
        while(opModeIsActive()) {
            gg.update();
            drive.setMotorPowersFromGamepad(gg,1.0,false,true);
        }
    }
}
