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
import org.firstinspires.ftc.teamcode.subsystems.Arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;

import java.util.ArrayList;

@Config
@Autonomous(name = "5 + 0")
public class Specinem5 extends LinearOpMode {

    Robot robot = null;

    // We'll store various multi-waypoint Paths here
    Path firstSample, secondSample, thirdSample, pickUP;
    Path secondPath, thirdPath, forthPath, fifthPath, sixthPath;
    Path PickUp2, PickUp3, PickUp5, PickUp4, PickUp6, PickUp7, PickUp8;

    Pose2d park = new Pose2d(-40,59,Math.toRadians(180));
    Pose2d startPose = new Pose2d(-10, 60.2, Math.toRadians(270));

    // Various "specimen" pickup or drop points
    Pose2d specinem1    = new Pose2d(-9, 35.5, Math.toRadians(270));
    Pose2d specinem1_1  = new Pose2d(-3.5443, 52, Math.toRadians(270));
    Pose2d specimen2    = new Pose2d(-6, 30, Math.toRadians(90));
    Pose2d specimen3    = new Pose2d(-4, 30, Math.toRadians(90));
    Pose2d specimen4    = new Pose2d(-1.2, 29.5, Math.toRadians(90));
    Pose2d specimen5    = new Pose2d(1, 29.5, Math.toRadians(90));
    Pose2d specimen6    = new Pose2d(-1, 29.5, Math.toRadians(90));

    Pose2d pickUpspot           = new Pose2d(-35 , 57.5, Math.toRadians(90));
    Pose2d transitionpickUpspot = new Pose2d(-36.4165, 48, Math.toRadians(90));

    public static double diff           = 125;
    public static double speciemExtension = 428;
    public static double extension1     = 427;
    public static double extension2     = 480;
    public static double extension3     = 425;

    public static double headingDegrees1 = 234.3;
    public static double headingDegrees2 = 224;
    public static double headingDegrees3 = 223.3;
    public static double specimenSlam    = 240;

    // We'll fill these at runtime
    public static Pose2d moveFirstPos1 = new Pose2d(-35.2481, 46.4915, Math.toRadians(headingDegrees1));
    public static Pose2d moveFirstPos2 = new Pose2d(-31.7,    45.107,  Math.toRadians(131));

    public static Pose2d moveSecondPos1 = new Pose2d(-40.3261, 43.5203, Math.toRadians(headingDegrees2));
    public static Pose2d moveSecondPos2 = new Pose2d(-37.3783, 40.0191, Math.toRadians(124));

    public static Pose2d moveThirdPos1  = new Pose2d(-50.9373, 42.2023, Math.toRadians(headingDegrees3));
    public static Pose2d moveThirdPos2  = new Pose2d(-32,      44,      Math.toRadians(120));

    // Another point used for transitions, etc.
    public static Pose2d transitionPoint = new Pose2d(-6, 44.5, Math.toRadians(114));

    // Subsystem states
    SPECIMEN specimen     = new SPECIMEN();
    SPECIMENGARD gard     = new SPECIMENGARD();
    INTAKING intaking     = new INTAKING();
    SPECIMENSLAM slam     = new SPECIMENSLAM();
    HIGHBASKET high       = new HIGHBASKET();

    // We'll build path arrays below
    ArrayList<Pose2d> pathforsample1 = new ArrayList<Pose2d>() {
        {
            add(transitionPoint);
            add(specimen2);
        }
    };
    ArrayList<Pose2d> pathforsample2 = new ArrayList<Pose2d>() {
        {
            add(transitionPoint);
            add(specimen3);
        }
    };
    ArrayList<Pose2d> pathforsample3 = new ArrayList<Pose2d>() {
        {
            add(transitionPoint);
            add(specimen4);
        }
    };
    ArrayList<Pose2d> fifth = new ArrayList<Pose2d>() {
        {
            add(transitionPoint);
            add(specimen5);
        }
    };
    ArrayList<Pose2d> sixth = new ArrayList<Pose2d>() {
        {
            add(transitionPoint);
            add(specimen6);
        }
    };
    ArrayList<Pose2d> pickUpPath = new ArrayList<Pose2d>() {
        {
            add(transitionpickUpspot);
            add(pickUpspot);
        }
    };

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.IS_AUTO = true;
        robot = new Robot(this, startPose);

        // Just in case we want to update these from Dashboard:
        moveFirstPos1 = new Pose2d(-35.24815500815084,46.491546390563485, Math.toRadians(headingDegrees1));
        moveFirstPos2 = new Pose2d(-31.7,            45.107,              Math.toRadians(131));
        moveSecondPos1 = new Pose2d(-40.32611035925197,43.52032999354085, Math.toRadians(headingDegrees2));
        moveSecondPos2 = new Pose2d(-37.37830070435532,40.01918234036664, Math.toRadians(124));
        moveThirdPos1  = new Pose2d(-50.9373077632874, 42.20239882581816, Math.toRadians(headingDegrees3));
        moveThirdPos2  = new Pose2d(-32,              44,                Math.toRadians(120));

        robot.start();

        // Build the Path objects
        firstSample  = new Path(pathforsample1);
        secondSample = new Path(pathforsample2);
        thirdSample  = new Path(pathforsample3);
        pickUP       = new Path(pickUpPath);

        secondPath = new Path(pathforsample1);
        forthPath  = new Path(pathforsample3);
        sixthPath  = new Path(sixth);
        fifthPath  = new Path(fifth);

        PickUp2 = new Path(pickUpPath);
        PickUp3 = new Path(pickUpPath);
        PickUp4 = new Path(pickUpPath);
        PickUp5 = new Path(pickUpPath);
        PickUp6 = new Path(pickUpPath);
        PickUp7 = new Path(pickUpPath);
        PickUp8 = new Path(pickUpPath);

        // Wait for start
        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Waiting for start...");
            telemetry.update();
        }
        if (isStopRequested()) {
            robot.stop();
            return;
        }

        // Begin autonomous routine
        placeSpecimen();
        moveSample1();
        moveSample2();
        moveSample3();
        placeSecondSpecimen();
        placeThirdSpecimen();
        placeFourSpecimen();
        placeFifthSpecimen();
        // placeSixthSpecimen(); // commented out in your code
        park();

        // Done
        robot.stop();
    }

    //////////////////////////////////////////////////////////////////////////////////
    // Below are your steps, updated to use setTargetPosition(...) with either Pose2d or Path
    //////////////////////////////////////////////////////////////////////////////////

    void placeSpecimen() {
        // 1) Single Pose2d move
        robot.drive.setTargetPosition(specinem1_1, false, true, 0.85);

        // 2) Arm movements
        robot.arm.setAutoTargetState(ArmState.SPECIMEN);
        robot.arm.changeExtension(speciemExtension);

        waitUntilIdleArmAndDrive();

        // Move again
        robot.drive.setTargetPosition(specinem1, false, true, 0.85);
        robot.arm.changeExtension(speciemExtension);

        waitUntilIdleArmAndDrive();

        // Adjust extension
        robot.arm.changeExtension(speciemExtension - diff);
        waitUntilIdleArm();

        // Open claw
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.1);
    }

    void moveSample1() {
        // Robot sets arm to INTAKING
        robot.arm.setAutoTargetState(ArmState.INTAKING);

        // Single Pose2d move
        robot.drive.setTargetPosition(moveFirstPos1, false, true, 0.7);
        waitUntilIdleArmAndDrive();

        robot.arm.setAutoTargetState(ArmState.INTAKING);
        robot.arm.changeExtension(extension1);
        waitUntilIdleArm();

        // Claw operations
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.sleep(0.2);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.15);
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;

        // Another single move
        robot.drive.setTargetPosition(moveFirstPos2, false, true, 0.7);
        waitUntilIdleDrive();

        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
    }

    void moveSample2() {
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;

        robot.drive.setTargetPosition(moveSecondPos1, false, true, 0.7);
        waitUntilIdleDrive();

        robot.arm.changeExtension(extension2);
        waitUntilIdleArm();

        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.15);
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;

        robot.drive.setTargetPosition(moveSecondPos2, false, true, 0.7);
        waitUntilIdleDrive();
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.1);
    }

    void moveSample3() {
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.arm.setAutoTargetState(ArmState.INTAKING);

        robot.drive.setTargetPosition(moveThirdPos1, false, true, 0.7);
        waitUntilIdleArmAndDrive();

        robot.arm.changeExtension(extension3);
        waitUntilIdleArm();

        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.sleep(0.1);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.2);
        robot.arm.clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;

        robot.drive.setTargetPosition(moveThirdPos2, false, true, 0.7);
        waitUntilIdleDrive();

        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.1);
    }

    void placeSecondSpecimen() {
        // Example of path usage (if you had a path for 'PickUp6')
        robot.arm.setAutoTargetState(ArmState.SPECIMENGARD);
        waitUntilIdleArm();

        // If 'PickUp6' is a Path
        robot.drive.setTargetPosition(PickUp6, false, true, 0.7);
        waitUntilIdleDrive();

        // Grabbing logic
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.15);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;

        robot.drive.setTargetPosition(firstSample, false, true, 0.7);
        waitUntilIdleDrive();

        robot.arm.setAutoTargetState(ArmState.SPECIMENSLAM);
        waitUntilIdleArm();

        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
    }

    void placeThirdSpecimen() {
        // Robot drives using a path
        robot.drive.setTargetPosition(PickUp5, false, true, 0.7);
        robot.arm.setAutoTargetState(ArmState.SPECIMENGARD);
        waitUntilIdleArmAndDrive();

        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.15);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;

        // Another path
        robot.drive.setTargetPosition(forthPath, false, true, 0.7);
        waitUntilIdleDrive();

        robot.sleep(0.1);
        robot.arm.setAutoTargetState(ArmState.SPECIMENSLAM);
        waitUntilIdleArm();

        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
    }

    void placeFourSpecimen() {
        // Reusing 'PickUp4' as a Path
        robot.drive.setTargetPosition(PickUp4, false, true, 0.7);
        robot.arm.setAutoTargetState(ArmState.SPECIMENGARD);
        waitUntilIdleArmAndDrive();

        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.15);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;

        robot.drive.setTargetPosition(forthPath, false, true, 0.7);
        waitUntilIdleDrive();

        robot.sleep(0.1);
        robot.arm.setAutoTargetState(ArmState.SPECIMENSLAM);
        waitUntilIdleArm();

        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
    }

    void placeFifthSpecimen() {
        // Another path usage
        robot.drive.setTargetPosition(PickUp7, false, true, 0.7);
        robot.arm.setAutoTargetState(ArmState.SPECIMENGARD);
        waitUntilIdleArmAndDrive();

        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.15);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;

        robot.drive.setTargetPosition(fifthPath, false, true, 0.7);
        waitUntilIdleDrive();

        robot.sleep(0.1);
        robot.arm.setAutoTargetState(ArmState.SPECIMENSLAM);
        waitUntilIdleArm();

        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
    }

    void placeSixthSpecimen() {
        // Another path usage
        robot.drive.setTargetPosition(PickUp8, false, true, 0.7);
        robot.arm.setAutoTargetState(ArmState.SPECIMENGARD);
        waitUntilIdleArmAndDrive();

        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.15);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;

        robot.drive.setTargetPosition(sixthPath, false, true, 0.7);
        waitUntilIdleDrive();

        robot.sleep(0.1);
        robot.arm.setAutoTargetState(ArmState.SPECIMENSLAM);
        waitUntilIdleArm();

        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
    }

    void park() {
        // Single Pose2d, or you could build a Path for a multi-waypoint park
        robot.drive.setTargetPosition(park, false, true, 0.7);
        robot.arm.setAutoTargetState(ArmState.INTAKING);
        waitUntilIdleArmAndDrive();
    }

    ////////////////////////////////////////////////////////////////////////////////////
    // Helper methods to wait for Arm/Drive to finish
    ////////////////////////////////////////////////////////////////////////////////////

    /** Wait until the drive is IDLE. */
    private void waitUntilIdleDrive() {
        while (opModeIsActive() && !isStopRequested()
                && robot.drive.state != DriveTrain.STATE.IDLE) {
            telemetryOutput();
            robot.sleep(0.001);
        }
    }

    /** Wait until the Arm is IDLE. */
    private void waitUntilIdleArm() {
        while (opModeIsActive() && !isStopRequested()
                && robot.arm.currentState != Arm.FSMState.IDLE) {
            telemetryOutput();
            robot.sleep(0.001);
        }
    }

    /** Wait until both Arm & Drive are IDLE. */
    private void waitUntilIdleArmAndDrive() {
        while (opModeIsActive() && !isStopRequested()
                && (robot.drive.state != DriveTrain.STATE.IDLE
                || robot.arm.currentState != Arm.FSMState.IDLE)) {
            telemetryOutput();
            robot.sleep(0.001);
        }
    }

    private void telemetryOutput() {
        telemetry.addData("extension", robot.arm.extensionSubsystem.currentPos);
        telemetry.addData("robot arm", robot.arm.currentState);
        telemetry.addData("drive state", robot.drive.state);
        telemetry.addData("pose", robot.drive.getPose());
        telemetry.update();
    }
}
