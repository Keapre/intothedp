package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.DEFAUlT;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.HIGHBASKET;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.INTAKING;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.SPECIMEN;
import org.firstinspires.ftc.teamcode.Utils.geometry.Path;
import org.firstinspires.ftc.teamcode.Utils.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.P2Pdrive;

import java.util.ArrayList;

@Config
@Autonomous(name = "0+4")
public class JustSampleAuto extends LinearOpMode {

    Path first,parkPath;
    //Pose2d startPose = new Pose2d(32, 60.3, Math.toRadians(270));
    Pose2d startPose = new Pose2d(32, 60.3, Math.toRadians(270));
    Robot2 robot = null;
    Pose2d scoreBasket2 = new Pose2d(57,57.5,Math.toRadians(223));
    Pose2d scoreBasket2_2 = new Pose2d(57,56.7,Math.toRadians(235));
    Pose2d scoreBasket1 = new Pose2d(52,52,Math.toRadians(243));
    Pose2d scoreBasket1_1 = new Pose2d(52,52,Math.toRadians(243));

    Pose2d bucketFirst = new Pose2d(55,45,Math.toRadians(250));
    Pose2d transition = new Pose2d(23,46,Math.toRadians(265));
    Pose2d bucketSecond = new Pose2d(58,43,Math.toRadians(270));
    Pose2d bucketThird = new Pose2d(56.6,50,Math.toRadians(289));

    ArrayList<Pose> toPark = new ArrayList<Pose>();
    ArrayList<Pose> toBucket = new ArrayList<Pose>();


    INTAKING intaking = new INTAKING();
    HIGHBASKET high = new HIGHBASKET();
    SPECIMEN specinem = new SPECIMEN();

    public static double basketLength = 573;
    public static double firstBucketExtension = 352;
    public static double secondBucketExtension = 264;
    public static double parklength = 135;
    public static  double thirdBucketExtension = 520;
    Pose2d park = new Pose2d(25,9,Math.toRadians(180));


    public void placeSpecimen() {
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.drive.setTargetPose(scoreBasket1);

        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.arm.setAutoTargetState(high);
        robot.arm.changeDesiredExtension(basketLength);
        while(robot.arm.currentState != Arm.FSMState.IDLE  && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.sleep(0.2);
        robot.drive.setTargetPose(scoreBasket2);

        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.sleep(0.3);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.2);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.sleep(0.1);
        robot.drive.setTargetPose(scoreBasket1);
        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
    }
    public void firstBucket() {
        robot.arm.setAutoTargetState(intaking);
        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.drive.setTargetPose(bucketFirst);

        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE &&  opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.update();
            robot.sleep(0.001);
        }

        robot.arm.setAutoTargetState(intaking);
        robot.arm.changeDesiredExtension(firstBucketExtension);
        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }



        robot.sleep(0.3);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.1);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.drive.setTargetPose(scoreBasket1);
        robot.arm.setAutoTargetState(high);
        robot.arm.changeDesiredExtension(basketLength);
        while((robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.update();
            robot.sleep(0.001);
        }

        robot.sleep(0.1);

        robot.drive.setTargetPose(scoreBasket2);
        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.update();
            robot.sleep(0.001);
        }


        robot.sleep(0.1);

        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.2);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.sleep(0.1);
        robot.drive.setTargetPose(scoreBasket1);
        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }


    }

    public void secondBucket() {
        robot.arm.setAutoTargetState(intaking);
        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.drive.setTargetPose(bucketSecond);

        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE &&  opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.update();
            robot.sleep(0.001);
        }

        robot.arm.setAutoTargetState(intaking);
        robot.arm.changeDesiredExtension(secondBucketExtension);
        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }



        robot.sleep(0.3);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.1);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.drive.setTargetPose(scoreBasket1);
        robot.arm.setAutoTargetState(high);
        robot.arm.changeDesiredExtension(basketLength);
        while((robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.update();
            robot.sleep(0.001);
        }

        robot.sleep(0.1);

        robot.drive.setTargetPose(scoreBasket2);
        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.update();
            robot.sleep(0.001);
        }


        robot.sleep(0.1);

        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.2);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.sleep(0.1);
        robot.drive.setTargetPose(scoreBasket1);
        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }


    }

    public void thirdBasket() {
        robot.arm.setAutoTargetState(intaking);
        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.drive.setTargetPose(bucketThird);

        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE &&  opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.update();
            robot.sleep(0.001);
        }
        robot.arm.setAutoTargetState(intaking);
        robot.arm.changeDesiredExtension(thirdBucketExtension);
        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }



        robot.sleep(0.3);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.1);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.drive.setTargetPose(scoreBasket1);
        robot.arm.setAutoTargetState(high);
        robot.arm.changeDesiredExtension(basketLength);
        while((robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.update();
            robot.sleep(0.001);
        }

        robot.sleep(0.1);

        robot.drive.setTargetPose(scoreBasket2);
        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.update();
            robot.sleep(0.001);
        }


        robot.sleep(0.1);

        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.2);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.sleep(0.1);
        robot.drive.setTargetPose(scoreBasket1);
        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }


    }

    void park() {
        robot.arm.setAutoTargetState(intaking);
        while( robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;

        robot.sleep(0.1);
        robot.drive.setTargetPose(parkPath);
        robot.arm.setAutoTargetState(intaking);
        while((robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE ||  robot.arm.currentState!= Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot2(this,true,startPose);
        toPark.add(new Pose(new Pose2d(41,26,Math.toRadians(270))));
        toPark.add(new Pose(park));
        parkPath = new Path(toPark);
        toBucket.add(new Pose(44.1,50,Math.toRadians(234)));
        toBucket.add(new Pose(scoreBasket2));
        first = new Path(toBucket);
        robot.start();
        while (!isStarted()) {
            if (isStopRequested()) {
                robot.stop();
            }
        }

        if (isStopRequested()) {
            robot.stop();
        }

        placeSpecimen();
        firstBucket();
        secondBucket();
        thirdBasket();
        park();
        robot.stop();
    }
}
