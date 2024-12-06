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
@Autonomous(name = "Blue 1+3")
public class speciemSampleAuto extends LinearOpMode {

    Path first,parkPath;
    Pose2d startPose = new Pose2d(7, 60.5, Math.toRadians(270));
    Pose2d specimenDrop = new Pose2d(5, 36.5, Math.toRadians(270));
    Pose2d bucketFirst = new Pose2d(55,45,Math.toRadians(253));
    Pose2d transition = new Pose2d(23,46,Math.toRadians(265));
    Pose2d bucketSecond = new Pose2d(58,43,Math.toRadians(272));
    Pose2d bucketThird = new Pose2d(56.6,50,Math.toRadians(289));
    Pose2d trans = new Pose2d(5,48,Math.toRadians(270));

    Pose2d scoreBasket2 = new Pose2d(55.5,56,Math.toRadians(223));
    Pose2d scoreBasket2_2 = new Pose2d(57,56.7,Math.toRadians(235));
    Pose2d scoreBasket1 = new Pose2d(52,52,Math.toRadians(243));
    Pose2d scoreBasket1_1 = new Pose2d(52,52,Math.toRadians(243));
    SPECIMEN specimen = new SPECIMEN();
    INTAKING intaking = new INTAKING();
    INTAKING intaking2 = new INTAKING();
    INTAKING intaking3 = new INTAKING();
    DEFAUlT def = new DEFAUlT();
    public static double firstBucketExtension = 340;
    public static double secondBucketExtension = 260;
    public static  double thirdBucketExtension = 520;
    public static  double diff = 130;

    public static double basketLength = 560;
    public static  double speciemExtension = 400;
    HIGHBASKET high = new HIGHBASKET();
    HIGHBASKET high2 = new HIGHBASKET();
    HIGHBASKET high3 = new HIGHBASKET();
    Robot2 robot = null;
    ArrayList<Pose> toBucket = new ArrayList<Pose>();
    ArrayList<Pose> toPark = new ArrayList<Pose>();

    Pose2d park = new Pose2d(25,9,Math.toRadians(180));


    void placeSpecimen() {
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.arm.setAutoTargetState(specimen);
        robot.arm.changeDesiredExtension(speciemExtension);
        while(robot.arm.currentState != Arm.FSMState.IDLE  && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.drive.setTargetPose(specimenDrop);

        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }


//        while((robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("arm state",robot.arm.currentState);
//            telemetry.addData("drive state",robot.drive.driveMode);
//            Log.d("AutoInfo","arm state" + String.valueOf(robot.arm.currentState));
//            Log.d("AutoInfo","driveMode" + String.valueOf(robot.drive.driveMode));
//            Log.d("AutoInfo","pose" + robot.drive.pose.toString());
//            Log.d("AutoInfo","pitchPose" + String.valueOf(robot.arm.pitchSubsystem.getCurrentPos()));
//            Log.d("AutoInfo","extensionPose" + String.valueOf(robot.arm.extensionSubsystem.getCurrentPosition()));
//            Log.d("AutoInfo","pose" + robot.drive.pose.toString());
//            robot.sleep(0.001);
//        }
        robot.sleep(0.2);
        robot.arm.changeExtension(speciemExtension-diff);
        while(robot.arm.currentState != Arm.FSMState.IDLE && robot.arm.extensionSubsystem.getTimer()<500 && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.sleep(0.1);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.25);
        robot.arm.setAutoTargetState(intaking);
        while(robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
    }

    void firstBucket() {
        robot.drive.setTargetPose(first);

        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.arm.setAutoTargetState(intaking2);
        robot.arm.changeDesiredExtension(firstBucketExtension);
        while(robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.sleep(0.3);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.sleep(0.3);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.2);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.sleep(0.1);
        robot.drive.setTargetPose(scoreBasket1);
        robot.arm.setAutoTargetState(high2);
        robot.arm.changeDesiredExtension(basketLength);
        while((robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }

        robot.sleep(0.1);

        robot.drive.setTargetPose(scoreBasket2);
        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.addData("robot dt",robot.drive.driveMode);
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
    void secondBucket() {
        robot.drive.setTargetPose(bucketSecond);
        robot.arm.setAutoTargetState(intaking3);
        while((robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) &&  opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.update();
            robot.sleep(0.001);
        }

        robot.arm.setAutoTargetState(intaking2);
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
        robot.arm.setAutoTargetState(high2);
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
    void thirdBasket() {
        robot.drive.setTargetPose(bucketThird);
        robot.arm.setAutoTargetState(intaking3);
        while((robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) &&  opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }

        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.drive.currentPose);

            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.arm.setAutoTargetState(intaking3);
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
        robot.arm.setAutoTargetState(high2);
        robot.arm.changeDesiredExtension(basketLength);
        while((robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) &&  opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("pose",robot.drive.currentPose);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.sleep(0.2);


        robot.drive.setTargetPose(new Pose2d(scoreBasket2.position.x,scoreBasket2.position.x,scoreBasket2.heading.toDouble()));
        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("pose",robot.drive.currentPose);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.drive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }

        robot.sleep(0.1);
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.2);
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.sleep(0.1);
        robot.drive.setTargetPose(scoreBasket1);
        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
    }

    void park() {
        robot.arm.setAutoTargetState(intaking3);
        while( robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.drive.setTargetPose(parkPath);
        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.arm.setAutoTargetState(intaking);
        while( robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        toBucket.add(new Pose(transition));
        toBucket.add(new Pose(bucketFirst));
        toPark.add(new Pose(new Pose2d(41,26,Math.toRadians(270))));
        toPark.add(new Pose(park));
        first = new Path(toBucket);
        parkPath = new Path(toPark);
        robot = new Robot2(this,true,startPose);



        while (!isStarted()) {
            if (isStopRequested()) {
                robot.stop();
            }
        }
        robot.start();
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
