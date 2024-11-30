package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utils.geometry.Path;
import org.firstinspires.ftc.teamcode.Utils.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.P2Pdrive;

import java.util.ArrayList;

@Autonomous(name = "test")
public class Test extends LinearOpMode {

    Pose2d startPose = new Pose2d(7, 61, Math.toRadians(270));
    Pose2d specimenDrop = new Pose2d(3, 41, Math.toRadians(270));
    Pose2d bucketFirst = new Pose2d(54,50,Math.toRadians(260));
    Pose2d transition = new Pose2d(23,46,Math.toRadians(265));
    Pose2d bucketSecond = new Pose2d(57,45,Math.toRadians(270));
    Pose2d bucketThird = new Pose2d(57,45,Math.toRadians(295));

    ArrayList<Pose> toBucket = new ArrayList<Pose>();

    Pose2d park = new Pose2d(-55,-55,Math.toRadians(45));

    P2Pdrive dt;
    @Override
    public void runOpMode() throws InterruptedException {
        toBucket.add(new Pose(transition));
        toBucket.add(new Pose(bucketFirst));
        Path path = new Path(toBucket);
        dt = new P2Pdrive(hardwareMap,startPose,true);

        waitForStart();
        while (opModeIsActive()){
            dt.setTargetPose(specimenDrop);
            while(dt.driveMode == P2Pdrive.DriveMode.GO_TO_TARGET) {
                dt.update();
            }
            dt.setTargetPose(path);
            while(dt.driveMode == P2Pdrive.DriveMode.GO_TO_TARGET) {
                dt.update();
            }
            dt.setTargetPose(bucketFirst);
            while(dt.driveMode == P2Pdrive.DriveMode.GO_TO_TARGET) {
                dt.update();
            }

            dt.setTargetPose(bucketSecond);
            while(dt.driveMode == P2Pdrive.DriveMode.GO_TO_TARGET) {
                dt.update();
            }
            dt.setTargetPose(bucketThird);
            while(dt.driveMode == P2Pdrive.DriveMode.GO_TO_TARGET) {
                dt.update();
            }
//            sleep(350);
//            dt.setTargetPose(park);
//            while(dt.driveMode == P2Pdrive.DriveMode.GO_TO_TARGET) {
//                dt.update();
//            }
//            sleep(750);
            stop();
        }
    }
}
