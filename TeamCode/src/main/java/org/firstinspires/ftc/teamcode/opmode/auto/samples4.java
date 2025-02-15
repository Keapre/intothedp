package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.HIGHBASKET;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.INTAKING;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.SPECIMEN;
import org.firstinspires.ftc.teamcode.Utils.Globals;
import org.firstinspires.ftc.teamcode.Utils.geometry.Path;
import org.firstinspires.ftc.teamcode.Utils.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Arm.ArmState;

import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.P2Pdrive;

import java.util.ArrayList;

@Config
@Autonomous(name = "0+4")
public class samples4 extends LinearOpMode {

    Path first,parkPath;
    public static double heading1 = 250;
    public static double heading2 = 250;
    public static double heading3 = 250;
    //Pose2d startPose = new Pose2d(32, 60.3, Math.toRadians(270));
    Pose2d startPose = new Pose2d(32, 60.3, Math.toRadians(270));
    Robot robot = null;
    Pose2d scoreBasket2 = new Pose2d(57,57.5,Math.toRadians(223));
    Pose2d scoreBasket2_2 = new Pose2d(57,56.7,Math.toRadians(235));
    Pose2d scoreBasket1 = new Pose2d(52,52,Math.toRadians(243));
    Pose2d scoreBasket1_1 = new Pose2d(52,52,Math.toRadians(243));

    Pose2d bucketFirst = new Pose2d(55,45,Math.toRadians(heading1));
    Pose2d transition = new Pose2d(23,46,Math.toRadians(265));
    Pose2d bucketSecond = new Pose2d(58,43,Math.toRadians(heading2));
    Pose2d bucketThird = new Pose2d(56.6,50,Math.toRadians(heading3));

    ArrayList<Pose> toPark = new ArrayList<Pose>();
    ArrayList<Pose> toBucket = new ArrayList<Pose>();


    INTAKING intaking = new INTAKING();
    HIGHBASKET high = new HIGHBASKET();
    SPECIMEN specinem = new SPECIMEN();

    public static double basketLength = 950;
    public static double firstBucketExtension = 679;
    public static double secondBucketExtension = 591;
    public static double parklength = 135;
    public static  double thirdBucketExtension = 847;
    Pose2d park = new Pose2d(25,9,Math.toRadians(180));


    public void placeSpecimen() {
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.autoDrive.setTargetPose(scoreBasket1);

        while(robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.arm.setAutoTargetState(ArmState.HIGHBASKET);
        robot.arm.fakePid(basketLength);
        while(robot.arm.currentState != Arm.FSMState.IDLE  && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.sleep(0.2);
        robot.autoDrive.setTargetPose(scoreBasket2);

        while(robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
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
        robot.autoDrive.setTargetPose(scoreBasket1);
        while(robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.autoDrive.currentPose);

            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
    }
    public void firstBucket() {
        robot.arm.setTargetState(ArmState.INTAKING);
        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.autoDrive.setTargetPose(bucketFirst);

        while(robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE &&  opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.addData("pose",robot.autoDrive.currentPose);

            telemetry.update();
            robot.sleep(0.001);
        }

        robot.arm.setAutoTargetState(ArmState.INTAKING);
        robot.arm.fakePid(firstBucketExtension);
        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.autoDrive.currentPose);

            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }



        robot.sleep(0.3);
        robot.arm.clawSubsystem.tiltState = org.firstinspires.ftc.teamcode.subsystems.Arm.Claw.Claw.tiltMode.DOWN;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.1);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.autoDrive.setTargetPose(scoreBasket1);
        robot.arm.setAutoTargetState(ArmState.HIGHBASKET);
        robot.arm.fakePid(basketLength);
        while((robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.addData("pose",robot.autoDrive.currentPose);

            telemetry.update();
            robot.sleep(0.001);
        }

        robot.sleep(0.1);

        robot.autoDrive.setTargetPose(scoreBasket2);
        while(robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.addData("pose",robot.autoDrive.currentPose);

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
        robot.autoDrive.setTargetPose(scoreBasket1);
        while(robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.autoDrive.currentPose);

            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }


    }

    public void secondBucket() {
        robot.arm.setAutoTargetState(ArmState.INTAKING);
        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.autoDrive.setTargetPose(bucketSecond);

        while(robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE &&  opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.addData("pose",robot.autoDrive.currentPose);

            telemetry.update();
            robot.sleep(0.001);
        }

        robot.arm.setAutoTargetState(ArmState.INTAKING);
        robot.arm.changeDesiredExtension(secondBucketExtension);
        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.autoDrive.currentPose);

            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }



        robot.sleep(0.3);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.1);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.autoDrive.setTargetPose(scoreBasket1);
        robot.arm.setAutoTargetState(ArmState.HIGHBASKET);
        robot.arm.changeDesiredExtension(basketLength);
        while((robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.addData("pose",robot.autoDrive.currentPose);

            telemetry.update();
            robot.sleep(0.001);
        }

        robot.sleep(0.1);

        robot.autoDrive.setTargetPose(scoreBasket2);
        while(robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.addData("pose",robot.autoDrive.currentPose);

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
        robot.autoDrive.setTargetPose(scoreBasket1);
        while(robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.autoDrive.currentPose);

            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }


    }

    public void thirdBasket() {
        robot.arm.setAutoTargetState(ArmState.INTAKING);
        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.autoDrive.setTargetPose(bucketThird);

        while(robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE&&  opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.addData("pose",robot.autoDrive.currentPose);

            telemetry.update();
            robot.sleep(0.001);
        }
        robot.arm.setAutoTargetState(ArmState.INTAKING);
        robot.arm.changeDesiredExtension(thirdBucketExtension);
        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.autoDrive.currentPose);

            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }



        robot.sleep(0.3);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.1);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.autoDrive.setTargetPose(scoreBasket1);
        robot.arm.setAutoTargetState(ArmState.HIGHBASKET);
        robot.arm.changeDesiredExtension(basketLength);
        while((robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.addData("pose",robot.autoDrive.currentPose);

            telemetry.update();
            robot.sleep(0.001);
        }

        robot.sleep(0.1);

        robot.autoDrive.setTargetPose(scoreBasket2);
        while(robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.addData("pose",robot.autoDrive.currentPose);

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
        robot.autoDrive.setTargetPose(scoreBasket1);
        while(robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("pose",robot.autoDrive.currentPose);

            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }


    }

    void park() {
        robot.arm.setAutoTargetState(ArmState.INTAKING);
        while( robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;

        robot.sleep(0.1);
        robot.autoDrive.setTargetPose(parkPath);
        robot.arm.setAutoTargetState(ArmState.INTAKING);
        while((robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE ||  robot.arm.currentState!= Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {

        Globals.IS_AUTO = true;
        robot = new Robot(this,startPose);
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