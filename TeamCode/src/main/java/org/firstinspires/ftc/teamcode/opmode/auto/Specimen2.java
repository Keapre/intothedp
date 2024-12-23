//package org.firstinspires.ftc.teamcode.opmode.auto;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.RobotAuto;
//import org.firstinspires.ftc.teamcode.Utils.ArmStates.HIGHBASKET;
//import org.firstinspires.ftc.teamcode.Utils.ArmStates.INTAKING;
//import org.firstinspires.ftc.teamcode.Utils.ArmStates.SPECIMEN;
//import org.firstinspires.ftc.teamcode.Utils.ArmStates.SPECIMENGARD;
//import org.firstinspires.ftc.teamcode.Utils.ArmStates.SPECIMENSLAM;
//import org.firstinspires.ftc.teamcode.Utils.geometry.Path;
//import org.firstinspires.ftc.teamcode.Utils.geometry.Pose;
//import org.firstinspires.ftc.teamcode.subsystems.Arm.Arm;
//import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw;
//import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.P2Pdrive;
//
//import java.util.ArrayList;
//
//@Config
//@Autonomous(name = "5 + 0 preloaded cu spatele")
//public class Specimen2 extends LinearOpMode {
//    RobotAuto robot = null;
//    Path firstSample,secondSample,thirdSample,pickUP;
//    Path secondPath,thirdPath,forthPath,fifthPath,sixthPath;
//    Path PickUp2,PickUp3,PickUp5,PickUp4,PickUp6,PickUp7,PickUp8;
//    Pose2d park = new Pose2d(-40,59,Math.toRadians(180));
//
//    Pose2d startPose = new Pose2d(-5,57.5, Math.toRadians(90));
//    Pose2d specinem1 = new Pose2d(-5, 30, Math.toRadians(90));
//    Pose2d specinem1_1 = new Pose2d(-3.544303713821051, 52, Math.toRadians(270));
//    Pose2d specimen2 = new Pose2d(-3.5, 30, Math.toRadians(90));
//    Pose2d specimen3 = new Pose2d(-1.5, 30, Math.toRadians(90));
//    Pose2d specimen4 = new Pose2d(0, 29.5, Math.toRadians(90));
//    Pose2d specimen5 = new Pose2d(1.5, 29.5, Math.toRadians(90));
//
//    Pose2d pickUpspot = new Pose2d(-35 , 57.5, Math.toRadians(90));
//    Pose2d transitionpickUpspot = new Pose2d(-36.41655561792569, 48, Math.toRadians(90));
//
//
//    public static double diff = 125;
//
//    ArrayList<Pose> pathforsample1 = new ArrayList<Pose>() {
//        {
//            add(new Pose(transitionPoint));
//            add(new Pose(specimen2));
////            add(new Pose(new Pose2d(-37.80674251045768,10.826866570420153,Math.toRadians(270))));
////            add(new Pose(new Pose2d(-53.19720410925197,11.853723000353716,Math.toRadians(270))));
//        }
//    };
//    ArrayList<Pose> fifth= new ArrayList<Pose>() {
//        {
//            add(new Pose(transitionPoint));
//            add(new Pose(specimen5));
////            add(new Pose(new Pose2d(-37.80674251045768,10.826866570420153,Math.toRadians(270))));
////            add(new Pose(new Pose2d(-53.19720410925197,11.853723000353716,Math.toRadians(270))));
//        }
//    };
//    ArrayList<Pose> sixth= new ArrayList<Pose>() {
//        {
//            add(new Pose(transitionPoint));
//           // add(new Pose(specimen6));
////            add(new Pose(new Pose2d(-37.80674251045768,10.826866570420153,Math.toRadians(270))));
////            add(new Pose(new Pose2d(-53.19720410925197,11.853723000353716,Math.toRadians(270))));
//        }
//    };
//    ArrayList<Pose> pickUpPath = new ArrayList<Pose>() {
//        {
//            add(new Pose(transitionpickUpspot));
//            add(new Pose(pickUpspot));
////            add(new Pose(new Pose2d(-37.80674251045768,10.826866570420153,Math.toRadians(270))));
////            add(new Pose(new Pose2d(-53.19720410925197,11.853723000353716,Math.toRadians(270))));
//        }
//    };
//    ArrayList<Pose> pathforsample2 = new ArrayList<Pose>() {
//        {
//            add(new Pose(transitionPoint));
//            add(new Pose(specimen3));
//        }
//    };
//    ArrayList<Pose> pathforsample3 = new ArrayList<Pose>() {
//        {
//            add(new Pose(transitionPoint));
//            add(new Pose(specimen4));
//        }
//    };
//
//
//    public static double speciemExtension = 400;
//    public static double extension1 = 427;
//    public static double extension2 = 460;
//    public static double extension3 = 415;
//
//    public static double headingDegrees1 = 235.5;
//    public static double headingDegrees2 = 223.5;
//    public static double headingDegrees3 = 223;
//    public static double specimenSlam = 240;
//    SPECIMEN specimen = new SPECIMEN();
//    SPECIMENGARD gard = new SPECIMENGARD();
//    INTAKING intaking = new INTAKING();
//    public static Pose2d moveFirstPos1 = new Pose2d(-35.24815500815084,46.491546390563485,Math.toRadians(headingDegrees1));
//    public static Pose2d transitionPoint = new Pose2d(-6,44.5,Math.toRadians(114));
//    public static Pose2d moveFirstPos2 = new Pose2d(-31.7,45.107,Math.toRadians(131));
//
//    public static Pose2d moveSecondPos1 = new Pose2d(-40.32611035925197,43.52032999354085,Math.toRadians(headingDegrees2));
//    public static Pose2d moveSecondPos2 = new Pose2d(-37.37830070435532,40.01918234036664,Math.toRadians(124));
//
//    public static Pose2d moveThirdPos1 = new Pose2d(-50.9373077632874,42.20239882581816,Math.toRadians(headingDegrees3));
//    public static Pose2d moveThirdPos2 = new Pose2d(-32,44,Math.toRadians(120));
//    INTAKING intaking2 = new INTAKING();
//    INTAKING intaking3 = new INTAKING();
//    HIGHBASKET high = new HIGHBASKET();
//
//    SPECIMENSLAM slam = new SPECIMENSLAM();
//
//
//    HIGHBASKET high2 = new HIGHBASKET();
//    HIGHBASKET high3 = new HIGHBASKET();
//
//
//
//    void placeSpecimen() {
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
//        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;
//        robot.drive.setTargetPose(specinem1);
//        while (robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.sleep(0.1);
//        robot.arm.setAutoTargetState(slam);
//        while (robot.arm.currentState != Arm.FSMState.IDLE && robot.arm.extensionSubsystem.getTimer() < 500 && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.sleep(0.05);
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
//    }
//
//    void moveSample1() {
//        robot.arm.setAutoTargetState(intaking);
//        robot.drive.setTargetPose(moveFirstPos1);
//        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.arm.setAutoTargetState(intaking);
//        robot.arm.changeDesiredExtension(extension1);
//        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.sleep(0.1);
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
//        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
//        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
//        robot.sleep(0.2);
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
//        robot.sleep(0.15);
//        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
//        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
//        robot.drive.setTargetPose(moveFirstPos2);
//        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
//    }
//    void moveSample2() {
//        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
//        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
//        robot.drive.setTargetPose(moveSecondPos1);
//        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.arm.changeExtension(extension2);
//        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.sleep(0.1);
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
//        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
//        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
//
//        robot.sleep(0.25);
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
//        robot.sleep(0.15);
//        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
//        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
//        robot.drive.setTargetPose(moveSecondPos2);
//        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
//        robot.sleep(0.1);
//    }
//
//    void moveSample3() {
//        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
//        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
//        robot.arm.setAutoTargetState(intaking);
//        robot.drive.setTargetPose(moveThirdPos1);
//        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.arm.changeExtension(extension3);
//        while(robot.arm.currentState!= Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
//        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
//        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
//
//        robot.sleep(0.1);
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
//        robot.sleep(0.2);
//        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
//        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
//        robot.drive.setTargetPose(moveThirdPos2);
//        while(robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
//        robot.sleep(0.1);
//    }
//
//
//    void placeSecondSpecimen() {
//        robot.arm.setAutoTargetState(gard);
//        while (robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
//            robot.sleep(0.001);
//        }
//
//        robot.drive.setTargetPose(PickUp6);
//
//        while ( robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//
//        robot.sleep(0.25);
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
//        robot.sleep(0.15);
//        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;
//
//        robot.drive.setTargetPose(firstSample);
//
//        while (robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.arm.setAutoTargetState(slam);
//        while (robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.sleep(0.05);
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
//    }
//
//    void placeThirdSpecimen() {
//
//        robot.drive.setTargetPose(PickUp5);
//        robot.arm.setAutoTargetState(gard);
//        while ((robot.arm.currentState != Arm.FSMState.IDLE  || robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE) && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//
//        robot.sleep(0.25);
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
//        robot.sleep(0.15);
//        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;
//
//        robot.drive.setTargetPose(forthPath);
//
//        while (robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.sleep(0.1);
//        robot.arm.setAutoTargetState(slam);
//        while (robot.arm.currentState != Arm.FSMState.IDLE && robot.arm.extensionSubsystem.getTimer() < 500 && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.sleep(0.05);
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
//    }
//
//    void placeFourSpecimen() {
//        robot.drive.setTargetPose(PickUp4);
//        robot.arm.setAutoTargetState(gard);
//        while ((robot.arm.currentState != Arm.FSMState.IDLE  || robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE) && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//
//        robot.sleep(0.25);
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
//        robot.sleep(0.15);
//        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;
//
//        robot.drive.setTargetPose(forthPath);
//
//        while (robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.sleep(0.1);
//        robot.arm.setAutoTargetState(slam);
//        while (robot.arm.currentState != Arm.FSMState.IDLE && robot.arm.extensionSubsystem.getTimer() < 500 && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.sleep(0.05);
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
//    }
//
//    void placeFifthSpecimen() {
//        robot.drive.setTargetPose(PickUp7);
//        robot.arm.setAutoTargetState(gard);
//        while ((robot.arm.currentState != Arm.FSMState.IDLE  || robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE) && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//
//        robot.sleep(0.25);
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
//        robot.sleep(0.15);
//        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;
//
//        robot.drive.setTargetPose(fifthPath);
//
//        while (robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.sleep(0.1);
//        robot.arm.setAutoTargetState(slam);
//        while (robot.arm.currentState != Arm.FSMState.IDLE && robot.arm.extensionSubsystem.getTimer() < 500 && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.sleep(0.05);
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
//    }
//    void placeSixthSpecimen() {
//        robot.drive.setTargetPose(PickUp8);
//        robot.arm.setAutoTargetState(gard);
//        while ((robot.arm.currentState != Arm.FSMState.IDLE  || robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE) && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//
//        robot.sleep(0.25);
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
//        robot.sleep(0.15);
//        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;
//
//        robot.drive.setTargetPose(sixthPath);
//
//        while (robot.drive.driveMode!= P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.sleep(0.1);
//        robot.arm.setAutoTargetState(slam);
//        while (robot.arm.currentState != Arm.FSMState.IDLE && robot.arm.extensionSubsystem.getTimer() < 500 && opModeIsActive() && !isStopRequested()) {
//            telemetry.addData("extension",robot.arm.extensionSubsystem.currentPos);
//            telemetry.addData("robot arm",robot.arm.currentState);
//            telemetry.addData("robot dt",robot.drive.driveMode);
//            telemetry.update();
//            robot.sleep(0.001);
//        }
//        robot.sleep(0.05);
//        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
//    }
//    void park() {
//        robot.drive.setTargetPose(park);
//        robot.arm.setAutoTargetState(intaking);
//        while ((robot.drive.driveMode != P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
//            robot.sleep(0.001);
//        }
//    }
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot = new RobotAuto(this, true, startPose);
//        moveFirstPos1 = new Pose2d(-35.24815500815084,46.491546390563485,Math.toRadians(headingDegrees1));
//        moveFirstPos2 = new Pose2d(-31.7,45.107,Math.toRadians(131));
//
//        moveSecondPos1 = new Pose2d(-40.32611035925197,43.52032999354085,Math.toRadians(headingDegrees2));
//        moveSecondPos2 = new Pose2d(-37.37830070435532,40.01918234036664,Math.toRadians(124));
//
//        moveThirdPos1 = new Pose2d(-50.9373077632874,42.20239882581816,Math.toRadians(headingDegrees3));
//        moveThirdPos2 = new Pose2d(-32,44,Math.toRadians(120));
//
//        robot.start();
//
//        firstSample = new Path(pathforsample1);
//        secondSample = new Path(pathforsample2);
//        thirdSample = new Path(pathforsample2);
//        secondPath = new Path(pathforsample1);
//        forthPath = new Path(pathforsample3);
//        sixthPath = new Path(sixth);
//        fifthPath = new Path(fifth);
//        pickUP = new Path(pickUpPath);
//        PickUp2 = new Path(pickUpPath);
//        PickUp3 = new Path(pickUpPath);
//        PickUp4 = new Path(pickUpPath);
//        PickUp5 = new Path(pickUpPath);
//        PickUp6 = new Path(pickUpPath);
//        PickUp7 = new Path(pickUpPath);
//        PickUp8 = new Path(pickUpPath);
//
//        while (!isStarted()) {
//            if (isStopRequested()) {
//                robot.stop();
//            }
//        }
//        if (isStopRequested()) {
//            robot.stop();
//        }
//
//        placeSpecimen();
//        moveSample1();
//        moveSample2();
//        moveSample3();
//        placeSecondSpecimen();
//        placeThirdSpecimen();
//        placeFourSpecimen();
//        placeFifthSpecimen();
//        //placeSixthSpecimen();
//        park();
//        robot.stop();
//    }
//}
