package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Log;

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

@Autonomous(name = "1 + 0 (park)")
public class specinemPark extends LinearOpMode {
    Robot2 robot = null;
    Path firstSample,secondSample,thirdSample;
    Pose2d startPose = new Pose2d(-9.375536460576095, 60.81076314130167, Math.toRadians(270));
    Pose2d specinem1 = new Pose2d(-3.544303713821051, 36.5, Math.toRadians(270));
    Pose2d specimen2 = new Pose2d(-5.280821191982961, 36.5, Math.toRadians(270));
    Pose2d specimen3 = new Pose2d(-6.280821191982961, 36.5, Math.toRadians(270));
    Pose2d specimen4 = new Pose2d(-8.280821191982961, 36.5, Math.toRadians(270));

    Pose2d pickUpspot = new Pose2d(-23.71655561792569, 60.019627368356296, Math.toRadians(90));

    Pose2d moveFirstPos1 = new Pose2d(-34.3564635749877, 42.655495470903055, Math.toRadians(230));
    Pose2d moveFirstPos2 = new Pose2d(-34.3564635749877, 42.655495470903055, Math.toRadians(145));

    Pose2d moveSecondPos1 = new Pose2d(-47.535270630843996, 43.190216304749015, Math.toRadians(235));
    Pose2d moveSecondPos2 = new Pose2d(-47.535270630843996, 43.190216304749015, Math.toRadians(145));

    Pose2d moveThirdPos1 = new Pose2d(-53.65807240403544, 42.32089786078986, Math.toRadians(270));
    Pose2d moveThirdPos2 = new Pose2d(-53.65807240403544, 42.32089786078986, Math.toRadians(145));

    public static double moveExtension = 23000;
    public static double diff = 130;

    ArrayList<Pose> pathforsample1 = new ArrayList<Pose>() {
        {
            add(new Pose(new Pose2d(-24.6,43.6,Math.toRadians(220))));
            add(new Pose(new Pose2d(-44.9,14.5,Math.toRadians(90))));
            add(new Pose(new Pose2d(-44,51.6,Math.toRadians(90))));
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
            add(new Pose(new Pose2d(-59.6,16,Math.toRadians(90))));
            add(new Pose(new Pose2d(-50,53.3,Math.toRadians(78))));
        }
    };


    Pose2d park = new Pose2d(-40,59,Math.toRadians(180));
    public static double speciemExtension = 400;
    SPECIMEN specimen = new SPECIMEN();
    SPECIMENGARD gard = new SPECIMENGARD();
    INTAKING intaking = new INTAKING();
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
        robot.drive.setTargetPose(specimen4);

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
        robot.sleep(0.1);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.25);
        robot.arm.setAutoTargetState(intaking);
        while (robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
    }

    void moveSample1() {
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.drive.setTargetPose(firstSample);

        while (robot.drive.driveMode != P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
    }

    void moveSample2() {
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.drive.setTargetPose(secondSample);

        while (robot.drive.driveMode != P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
    }

    void moveSample3() {
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.drive.setTargetPose(thirdSample);

        while (robot.drive.driveMode != P2Pdrive.DriveMode.IDLE && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
    }

    void placeSecondSpecimen() {
        robot.drive.setTargetPose(pickUpspot);
        robot.arm.setAutoTargetState(gard);
        while ((robot.drive.driveMode != P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
            robot.sleep(0.001);
        }
        robot.sleep(0.15);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.2);
        robot.drive.setTargetPose(specimen2);
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

    void park() {
        robot.drive.setTargetPose(park);
        while ((robot.drive.driveMode != P2Pdrive.DriveMode.IDLE || robot.arm.currentState != Arm.FSMState.IDLE) && opModeIsActive() && !isStopRequested()) {
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
        while (!isStarted()) {
            if (isStopRequested()) {
                robot.stop();
            }
        }
        if (isStopRequested()) {
            robot.stop();
        }

        placeSpecimen();
        //moveSample1();
        //moveSample2();
        //moveSample3();
        //placeSecondSpecimen();
        //placeThirdSpecimen();
        //placeFourSpecimen();

        park();
        robot.sleep(0.2);
        robot.stop();
    }
}
