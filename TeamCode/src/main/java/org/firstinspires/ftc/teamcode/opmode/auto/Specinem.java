package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.HIGHBASKET;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.INTAKING;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.SPECIMEN;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.SPECIMENGARD;
import org.firstinspires.ftc.teamcode.Utils.geometry.Path;
import org.firstinspires.ftc.teamcode.Utils.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.P2Pdrive;

import java.util.ArrayList;

@Config
@Autonomous(name = "4 + 0")
public class Specinem extends LinearOpMode {
    Robot2 robot = null;
    Path firstSample,secondSample,thirdSample,pickUP;
    Pose2d startPose = new Pose2d(-9.375536460576095, 60.81076314130167, Math.toRadians(270));
    Pose2d specinem1 = new Pose2d(-3.544303713821051, 36.5, Math.toRadians(270));
    Pose2d specimen2 = new Pose2d(-5.280821191982961, 36.5, Math.toRadians(270));
    Pose2d specimen3 = new Pose2d(-6.280821191982961, 36.5, Math.toRadians(270));
    Pose2d specimen4 = new Pose2d(-8.280821191982961, 36.5, Math.toRadians(270));

    Pose2d pickUpspot = new Pose2d(-33.41655561792569, 58.019627368356296, Math.toRadians(90));


    public static double diff = 125;

    ArrayList<Pose> pathforsample1 = new ArrayList<Pose>() {
        {
            add(new Pose(new Pose2d(-34.61471197173351,45.134743517778055,Math.toRadians(243))));
            add(new Pose(new Pose2d(-37.80674251045768,10.826866570420153,Math.toRadians(270))));
            add(new Pose(new Pose2d(-53.19720410925197,11.853723000353716,Math.toRadians(270))));
        }
    };
    ArrayList<Pose> pathforsample2 = new ArrayList<Pose>() {
        {
            add(new Pose(new Pose2d(-54.4,16.5,Math.toRadians(90))));
            add(new Pose(new Pose2d(-50.2,57,Math.toRadians(90))));
        }
    };
    ArrayList<Pose> pathforsample3 = new ArrayList<Pose>() {
        {
            add(new Pose(new Pose2d(-50.83489846056841,33.149077648252955,Math.toRadians(260))));
            add(new Pose(new Pose2d(-54.76532222717766,11.480834240049829,Math.toRadians(255))));
            add(new Pose(new Pose2d(-62.808352492925685,9.705846591258613,Math.toRadians(270))));
        }
    };

    ArrayList<Pose> pathToPickUp = new ArrayList<Pose>() {
        {
            add(new Pose(new Pose2d(-12.3,42.7,Math.toRadians(90))));
            add(new Pose(pickUpspot));
        }
    };

    public static double speciemExtension = 400;
    public static double extension1 = 335;
    public static double extension2 = 360;
    public static double extension3 = 590;

    public static double headingDegrees1 = 237;
    public static double headingDegrees2 = 237;
    public static double headingDegrees3 = 237;
    SPECIMEN specimen = new SPECIMEN();
    SPECIMENGARD gard = new SPECIMENGARD();
    INTAKING intaking = new INTAKING();
    public static Pose2d moveFirstPos1 = new Pose2d(-34.24815500815084,42.491546390563485,Math.toRadians(headingDegrees1));
    public static Pose2d moveFirstPos2 = new Pose2d(-31.0,47.107,Math.toRadians(143));

    public static Pose2d moveSecondPos1 = new Pose2d(-45.32611035925197,43.52032999354085,Math.toRadians(headingDegrees2));
    public static Pose2d moveSecondPos2 = new Pose2d(-38.37830070435532,47.01918234036664,Math.toRadians(140));

    public static Pose2d moveThirdPos1 = new Pose2d(-50.9373077632874,40.20239882581816,Math.toRadians(headingDegrees3));
    public static Pose2d moveThirdPos2 = new Pose2d(-39.22288693405512,46.63408414585384,Math.toRadians(138));
    INTAKING intaking2 = new INTAKING();
    INTAKING intaking3 = new INTAKING();
    HIGHBASKET high = new HIGHBASKET();


    HIGHBASKET high2 = new HIGHBASKET();
    HIGHBASKET high3 = new HIGHBASKET();


    void placeSpecimen() {
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.arm.setAutoTargetState(specimen);
        robot.arm.changeDesiredExtension(speciemExtension);
        while (robot.arm.currentState != Arm.FSMState.IDLE) {
            telemetry.addData("extension", robot.arm.extensionSubsystem.currentPos);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.drive.setTargetPose(specinem1);

        while (robot.drive.driveMode != P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
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
        robot.arm.changeExtension(speciemExtension - diff);
        while (robot.arm.currentState != Arm.FSMState.IDLE && robot.arm.extensionSubsystem.getTimer() < 500 && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.1);
        robot.arm.setAutoTargetState(intaking);
        while (robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
    }

    Pose2d dropSample = new Pose2d(-53.8,53,Math.toRadians(270));
    void moveSample1() {
        robot.drive.setTargetPose(firstSample);
        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.drive.setTargetPose(dropSample);
        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.sleep(0.1);
    }
    void moveSample2() {
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.drive.setTargetPose(moveSecondPos1);
        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.arm.changeExtension(extension2);
        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.sleep(0.2);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.VERTICAL;

        robot.sleep(0.3);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.2);
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.drive.setTargetPose(moveSecondPos2);
        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.sleep(0.2);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.1);
    }

    Pose2d thirdDrop = new Pose2d(-62.808352492925685,53,Math.toRadians(270));
    Pose2d thirdPickUP = new Pose2d(-54.64,53,Math.toRadians(242));
    void moveSample3() {
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.drive.setTargetPose(thirdPickUP);
        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.arm.changeExtension(extension3);
        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.sleep(0.2);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.2);
        robot.arm.setAutoTargetState(intaking);
        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }

        robot.drive.setTargetPose(new Pose2d(dropSample.position.x,dropSample.position.y,Math.toRadians(90)));
        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
    }

    void placeSecondSpecimen() {


        robot.drive.setTargetPose(pickUP);
        robot.arm.setAutoTargetState(gard);
        while ((robot.arm.currentState != Arm.FSMState.IDLE  || robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE) && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }

        robot.sleep(0.15);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.2);

        robot.drive.setTargetPose(specimen2);
        robot.arm.setAutoTargetState(specimen);
        robot.arm.changeDesiredExtension(speciemExtension);
        while ((robot.arm.currentState != Arm.FSMState.IDLE  || robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE) && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.sleep(0.2);
        robot.arm.changeExtension(speciemExtension - diff);
        while (robot.arm.currentState != Arm.FSMState.IDLE && robot.arm.extensionSubsystem.getTimer() < 500 && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.sleep(0.1);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.25);
        robot.arm.setAutoTargetState(intaking);
        while (robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
    }

    void placeThirdSpecimen() {
        robot.drive.setTargetPose(pickUpspot);
        robot.arm.setAutoTargetState(gard);
        while ((robot.drive.driveMode != P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.sleep(0.15);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.2);
        robot.drive.setTargetPose(specimen3);
        robot.arm.setAutoTargetState(specimen);
        robot.arm.changeDesiredExtension(speciemExtension);
        while ((robot.drive.driveMode != P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.sleep(0.2);
        robot.arm.changeExtension(speciemExtension - diff);
        while (robot.arm.currentState != Arm.FSMState.IDLE && robot.arm.extensionSubsystem.getTimer() < 500 && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.sleep(0.1);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.25);
        robot.arm.setAutoTargetState(intaking);
        while (robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.arm.setAutoTargetState(intaking);
        ElapsedTime time = null;
        while (robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
    }

    void placeFourSpecimen() {
        robot.drive.setTargetPose(pickUpspot);
        robot.arm.setAutoTargetState(gard);
        while ((robot.drive.driveMode != P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.sleep(0.15);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.2);
        robot.drive.setTargetPose(specimen4);
        robot.arm.setAutoTargetState(specimen);
        robot.arm.changeDesiredExtension(speciemExtension);
        while ((robot.drive.driveMode != P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.sleep(0.2);
        robot.arm.changeExtension(speciemExtension - diff);
        while (robot.arm.currentState != Arm.FSMState.IDLE && robot.arm.extensionSubsystem.getTimer() < 500 && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.sleep(0.1);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.25);
        robot.arm.setAutoTargetState(intaking);
        while (robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.arm.setAutoTargetState(intaking);
        ElapsedTime time = null;
        while (robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot2(this, true, startPose);


        robot.start();

        firstSample = new Path(pathforsample1);
        secondSample = new Path(pathforsample2);
        thirdSample = new Path(pathforsample3);
        pickUP = new Path(pathToPickUp);

        while (!isStarted()) {
            if (isStopRequested()) {
                robot.stop();
            }
        }
        if (isStopRequested()) {
            robot.stop();
        }

        placeSpecimen();
        moveSample1();
        moveSample3();
        //placeSecondSpecimen();
        //placeThirdSpecimen();
        //placeFourSpecimen();

        robot.sleep(0.2);
        robot.stop();
    }
}
