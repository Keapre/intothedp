package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.HIGHBASKET;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.INTAKING;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.SPECIMEN;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.SPECIMENGARD;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.SPECIMENSLAM;
import org.firstinspires.ftc.teamcode.Utils.Globals;
import org.firstinspires.ftc.teamcode.Utils.geometry.Path;
import org.firstinspires.ftc.teamcode.Utils.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants;
import org.firstinspires.ftc.teamcode.subsystems.Arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.P2Pdrive;

import java.util.ArrayList;

@Config
@Autonomous(name = "5 + 0")
public class Specinem5 extends LinearOpMode {
    Robot robot = null;
    Path xx;
    Path firstSample,secondSample,thirdSample,pickUP;
    Path secondPath,thirdPath,forthPath,fifthPath,sixthPath;
    Path PickUp2,PickUp3,PickUp5,PickUp4,PickUp6,PickUp7,PickUp8;
    Pose2d park = new Pose2d(-40,59,Math.toRadians(180));

    Pose2d startPose = new Pose2d(-16.37046153151144, 63, Math.toRadians(180));
    Pose2d specinem1 = new Pose2d(-11.225201464074804, 49, Math.toRadians(113));
    Pose2d specinem1_1 = new Pose2d(-9.178335835614542, 33, Math.toRadians(90));
    Pose2d specimen2 = new Pose2d(-6, 33, Math.toRadians(90));
    Pose2d specimen3 = new Pose2d(-4, 33, Math.toRadians(90));
    Pose2d specimen4 = new Pose2d(-1.2, 33, Math.toRadians(90));
    Pose2d specimen5 = new Pose2d(1, 33, Math.toRadians(90));
    Pose2d specimen6 = new Pose2d(-3.5, 33,Math.toRadians(90));

    Pose2d pickUpspot = new Pose2d(-42 , 57.5, Math.toRadians(90));
    Pose2d transitionpickUpspot = new Pose2d(-41, 48, Math.toRadians(90));


    public static double diff = 125;

    ArrayList<Pose> move1 = new ArrayList<Pose>() {
        {
            add(new Pose(new Pose2d(-36.12287386195867,39.12287386195867,Math.toRadians(90))));
            add(new Pose(new Pose2d(-36.92312886395792,10.5,Math.toRadians(90))));
            add(new Pose(new Pose2d(-50.5,10.5,Math.toRadians(90))));
            add(new Pose(new Pose2d(-48.5,55   ,Math.toRadians(90))));
        }
    };
    ArrayList<Pose> move2 = new ArrayList<Pose>() {
        {
            add(new Pose(new Pose2d(-48.82068543922244,10.5,Math.toRadians(90))));
            add(new Pose(new Pose2d(-58.42485563022884,10.5,Math.toRadians(90))));
            add(new Pose(new Pose2d(-57,53.456901490218996,Math.toRadians(90))));
        }
    };
    ArrayList<Pose> move3 = new ArrayList<Pose>() {
        {
            add(new Pose(new Pose2d(-57,10.5,Math.toRadians(90))));
            add(new Pose(new Pose2d(-64.0570250984252,10.5,Math.toRadians(90))));
            add(new Pose(new Pose2d(-64.07845853069636,52,Math.toRadians(90))));
        }
    };

    ArrayList<Pose> pathforsample1 = new ArrayList<Pose>() {
        {
            add(new Pose(transitionPoint));
            add(new Pose(specimen2));
//            add(new Pose(new Pose2d(-37.80674251045768,10.826866570420153,Math.toRadians(270))));
//            add(new Pose(new Pose2d(-53.19720410925197,11.853723000353716,Math.toRadians(270))));
        }
    };

    ArrayList<Pose> pathforsamplexx = new ArrayList<Pose>() {
        {
            add(new Pose(transitionPoint));
            add(new Pose(specimen2));
//            add(new Pose(new Pose2d(-37.80674251045768,10.826866570420153,Math.toRadians(270))));
//            add(new Pose(new Pose2d(-53.19720410925197,11.853723000353716,Math.toRadians(270))));
        }
    };
    ArrayList<Pose> fifth= new ArrayList<Pose>() {
        {
            add(new Pose(transitionPoint));
            add(new Pose(specimen5));
//            add(new Pose(new Pose2d(-37.80674251045768,10.826866570420153,Math.toRadians(270))));
//            add(new Pose(new Pose2d(-53.19720410925197,11.853723000353716,Math.toRadians(270))));
        }
    };
    ArrayList<Pose> sixth= new ArrayList<Pose>() {
        {
            add(new Pose(transitionPoint));
            add(new Pose(specimen6));
//            add(new Pose(new Pose2d(-37.80674251045768,10.826866570420153,Math.toRadians(270))));
//            add(new Pose(new Pose2d(-53.19720410925197,11.853723000353716,Math.toRadians(270))));
        }
    };
    ArrayList<Pose> pickUpPath = new ArrayList<Pose>() {
        {
            add(new Pose(transitionpickUpspot));
            add(new Pose(pickUpspot));
//            add(new Pose(new Pose2d(-37.80674251045768,10.826866570420153,Math.toRadians(270))));
//            add(new Pose(new Pose2d(-53.19720410925197,11.853723000353716,Math.toRadians(270))));
        }
    };
    ArrayList<Pose> pathforsample2 = new ArrayList<Pose>() {
        {
            add(new Pose(transitionPoint));
            add(new Pose(specimen3));
        }
    };
    ArrayList<Pose> pathforsample3 = new ArrayList<Pose>() {
        {
            add(new Pose(transitionPoint));
            add(new Pose(specimen4));
        }
    };


    public static double speciemExtension = 428;
    public static double extension1 = 427;
    public static double extension2 = 480;
    public static double extension3 = 425;

    public static double headingDegrees1 = 234.3;
    public static double headingDegrees2 = 224;
    public static double headingDegrees3 = 223.3;
    public static double specimenSlam = 240;
    SPECIMEN specimen = new SPECIMEN();
    SPECIMENGARD gard = new SPECIMENGARD();
    INTAKING intaking = new INTAKING();
    public static Pose2d moveFirstPos1 = new Pose2d(-35.24815500815084,46.491546390563485,Math.toRadians(headingDegrees1));
    public static Pose2d transitionPoint = new Pose2d(-6,44.5,Math.toRadians(114));
    public static Pose2d moveFirstPos2 = new Pose2d(-31.7,45.107,Math.toRadians(131));

    public static Pose2d moveSecondPos1 = new Pose2d(-40.32611035925197,43.52032999354085,Math.toRadians(headingDegrees2));
    public static Pose2d moveSecondPos2 = new Pose2d(-37.37830070435532,40.01918234036664,Math.toRadians(124));

    public static Pose2d moveThirdPos1 = new Pose2d(-50.9373077632874,42.20239882581816,Math.toRadians(headingDegrees3));
    public static Pose2d moveThirdPos2 = new Pose2d(-32,44,Math.toRadians(120));
    INTAKING intaking2 = new INTAKING();
    INTAKING intaking3 = new INTAKING();
    HIGHBASKET high = new HIGHBASKET();

    SPECIMENSLAM slam = new SPECIMENSLAM();


    HIGHBASKET high2 = new HIGHBASKET();
    HIGHBASKET high3 = new HIGHBASKET();



    void placeSpecimen() {
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;

        robot.autoDrive.setTargetPose(specinem1_1);
        robot.arm.setAutoTargetState(ArmState.HIGHBASKET);
        while ((robot.autoDrive.driveMode != P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.01);
        }
        robot.sleep(0.25);
        robot.arm.fakePid( placeSpecimenDelta);
        while (robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension", robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }

        robot.sleep(0.1);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.1);
    }

    void moveSample12() {
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.autoDrive.setTargetPose(firstSample);

        robot.arm.setAutoTargetState(ArmState.INTAKING);

        while ((robot.autoDrive.driveMode != P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
    }

    void moveSample22() {
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.autoDrive.setTargetPose(secondSample);

        while (robot.autoDrive.driveMode != P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
    }

    void moveSample32() {
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.autoDrive.setTargetPose(thirdSample);

        while (robot.autoDrive.driveMode != P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
    }

    void moveSample1() {
        robot.arm.setAutoTargetState(ArmState.INTAKING);
        robot.autoDrive.setTargetPose(moveFirstPos1);
        while(robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE && robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.arm.setAutoTargetState(ArmState.INTAKING);
        robot.arm.changeExtension(extension1);
        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.01);
        }
        robot.sleep(0.1);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.sleep(0.2);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.15);
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.autoDrive.setTargetPose(moveFirstPos2);
        while(robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.01);
        }
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
    }
    void moveSample2() {
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.autoDrive.setTargetPose(moveSecondPos1);
        while(robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.01);
        }
        robot.arm.changeExtension(extension2);
        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.01);
        }
        robot.sleep(0.1);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;

        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.15);
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.autoDrive.setTargetPose(moveSecondPos2);
        while(robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.1);
    }

    void moveSample3() {
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.arm.setAutoTargetState(ArmState.INTAKING);
        robot.autoDrive.setTargetPose(moveThirdPos1);
        while(robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE && robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.arm.changeExtension(extension3);
        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;

        robot.sleep(0.1);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.2);
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.autoDrive.setTargetPose(moveThirdPos2);
        while(robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.1);
    }


    void placeSecondSpecimen() {
        pickUP.reset();
        robot.autoDrive.setTargetPose(PickUp6);
        robot.arm.setAutoTargetState(ArmState.SPECIMENGARD);
        while ((robot.arm.currentState != Arm.FSMState.IDLE  || robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE)&& opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }

        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.15);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;

        robot.autoDrive.setTargetPose(xx);
        robot.arm.setAutoTargetState(ArmState.HIGHBASKET);
        while ((robot.arm.currentState != Arm.FSMState.IDLE  || robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE) && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.sleep(0.15);
        robot.arm.fakePid(placeSpecimenDelta);
        while (robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.sleep(0.05);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
    }
    public static double placeSpecimenDelta = 250;

    void placeThirdSpecimen() {
        pickUP.reset();
        robot.autoDrive.setTargetPose(PickUp5);
        robot.arm.setAutoTargetState(ArmState.SPECIMENGARD);
        while ((robot.arm.currentState != Arm.FSMState.IDLE  || robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE) && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.01);
        }

        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.15);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;

        robot.autoDrive.setTargetPose(forthPath);
        robot.arm.setAutoTargetState(ArmState.HIGHBASKET);
        while ((robot.arm.currentState != Arm.FSMState.IDLE  || robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE) && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.sleep(0.15);
        robot.arm.fakePid(placeSpecimenDelta);
        while (robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.sleep(0.05);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
    }

    void placeFourSpecimen() {
        robot.autoDrive.setTargetPose(PickUp4);
        robot.arm.setAutoTargetState(ArmState.SPECIMENGARD);
        while ((robot.arm.currentState != Arm.FSMState.IDLE  || robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE) && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }

        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.15);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;

        sixthPath = new Path(sixth);
        robot.autoDrive.setTargetPose(sixthPath);
        robot.arm.setAutoTargetState(ArmState.HIGHBASKET);
        while ((robot.arm.currentState != Arm.FSMState.IDLE  || robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE) && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.sleep(0.15);
        robot.arm.fakePid(placeSpecimenDelta);
        while (robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.sleep(0.05);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
    }

    void placeFifthSpecimen() {
        robot.autoDrive.setTargetPose(PickUp7);
        robot.arm.setAutoTargetState(ArmState.SPECIMENGARD);
        while ((robot.arm.currentState != Arm.FSMState.IDLE  || robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE) && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }

        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.15);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;

        fifthPath = new Path(fifth);
        robot.autoDrive.setTargetPose(fifthPath);
        robot.arm.setAutoTargetState(ArmState.HIGHBASKET);
        while ((robot.arm.currentState != Arm.FSMState.IDLE  || robot.autoDrive.driveMode!= P2Pdrive.DriveMode.IDLE) && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.sleep(0.15);
        robot.arm.fakePid(placeSpecimenDelta);
        while (robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
            telemetry.addData("robot arm",robot.arm.currentState);
            telemetry.addData("robot dt",robot.autoDrive.driveMode);
            telemetry.update();
            robot.sleep(0.001);
        }
        robot.sleep(0.05);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
    }
    void park() {
        
        robot.autoDrive.setTargetPose(park);
        robot.arm.setAutoTargetState(ArmState.INTAKING);
        while ((robot.autoDrive.driveMode != P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Globals.IS_AUTO = true;
        robot = new Robot(this, startPose);
        moveFirstPos1 = new Pose2d(-35.24815500815084,46.491546390563485,Math.toRadians(headingDegrees1));
        moveFirstPos2 = new Pose2d(-31.7,45.107,Math.toRadians(131));

        moveSecondPos1 = new Pose2d(-40.32611035925197,43.52032999354085,Math.toRadians(headingDegrees2));
        moveSecondPos2 = new Pose2d(-37.37830070435532,40.01918234036664,Math.toRadians(124));

        moveThirdPos1 = new Pose2d(-50.9373077632874,42.20239882581816,Math.toRadians(headingDegrees3));
        moveThirdPos2 = new Pose2d(-32,44,Math.toRadians(120));
        robot.start();
        robot.arm.changePitch(ArmConstants.START_PITCH);
        xx = new Path(pathforsamplexx);
        firstSample = new Path(move1);
        secondSample = new Path(move2);
        thirdSample = new Path(move3);
        secondPath = new Path(pathforsample1);
        forthPath = new Path(pathforsample3);
        sixthPath = new Path(sixth);
        fifthPath = new Path(fifth);
        pickUP = new Path(pickUpPath);
        PickUp2 = new Path(pickUpPath);
        PickUp3 = new Path(pickUpPath);
        PickUp4 = new Path(pickUpPath);
        PickUp5 = new Path(pickUpPath);
        PickUp6 = new Path(pickUpPath);
        PickUp7 = new Path(pickUpPath);
        PickUp8 = new Path(pickUpPath);

        while (!isStarted()) {
            if (isStopRequested()) {
                robot.stop();
            }
        }
        if (isStopRequested()) {
            robot.stop();
        }

        placeSpecimen();
        moveSample12();
        moveSample22();
        moveSample32();
        placeSecondSpecimen();
        placeThirdSpecimen();
        placeFourSpecimen();
        placeFifthSpecimen();
        park();
        robot.stop();
    }
}