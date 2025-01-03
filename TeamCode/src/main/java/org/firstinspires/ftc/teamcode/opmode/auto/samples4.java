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
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.DriveTrain;

import java.util.ArrayList;

@Config
@Autonomous(name = "0+4")
public class samples4 extends LinearOpMode {

    // Predefined Path objects
    Path first, parkPath;

    // Starting pose
    Pose2d startPose = new Pose2d(32, 60.3, Math.toRadians(270));

    // Some key points/poses
    Pose2d scoreBasket1  = new Pose2d(52,52, Math.toRadians(243));
    Pose2d scoreBasket2  = new Pose2d(57,57.5,Math.toRadians(223));

    // Additional bucket pickup poses
    Pose2d bucketFirst   = new Pose2d(55,45,Math.toRadians(250));
    Pose2d bucketSecond  = new Pose2d(58,43,Math.toRadians(270));
    Pose2d bucketThird   = new Pose2d(56.6,50,Math.toRadians(289));

    // Park pose
    Pose2d park = new Pose2d(25,9,Math.toRadians(180));

    // Arm state objects
    INTAKING intaking = new INTAKING();
    HIGHBASKET high   = new HIGHBASKET();
    SPECIMEN specinem = new SPECIMEN();

    // Extension & other numeric configs
    public static double firstBucketExtension  = 345;
    public static double secondBucketExtension = 264;
    public static double basketLength          = 570;
    public static double thirdBucketExtension  = 520;

    Robot robot = null;

    // We'll store points for multi-step moves in lists
    ArrayList<Pose2d> toPark    = new ArrayList<>();
    ArrayList<Pose2d> toBucket  = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        Globals.IS_AUTO = true;
        // Initialize the robot with the given start pose
        robot = new Robot(this, startPose);

        // Build a path for parking (two waypoints)
        toPark.add((new Pose2d(41, 26, Math.toRadians(270))));
        toPark.add(park);
        parkPath = new Path(toPark);

        // Build a path for going to the bucket (example)
        toBucket.add(new Pose2d(44.1, 50, Math.toRadians(234)));
        toBucket.add(scoreBasket2);
        first = new Path(toBucket);

        robot.start();

        // Wait for start
        while (!isStarted() && !isStopRequested()) {
            // Optional: telemetry, etc.
        }
        if (isStopRequested()) {
            robot.stop();
            return;
        }

        // Autonomous routine
        placeSpecimen();
        firstBucket();
        secondBucket();
        thirdBasket();
        park();

        robot.stop();
    }


    public void placeSpecimen() {
        // Single-point drive to 'scoreBasket1'
        robot.drive.setTargetPosition(scoreBasket1, false, true, 0.85);
        waitUntilIdleDrive();

        // Raise the arm
        robot.arm.setAutoTargetState(ArmState.HIGHBASKET);
        robot.arm.changeExtension(basketLength);
        waitUntilIdleArmAndDrive();

        // Move to 'scoreBasket2' (single point)
        robot.drive.setTargetPosition(scoreBasket2, true, true, 0.7);
        waitUntilIdleDrive();

        // Perform tilt and drop
        robot.sleep(0.3);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos  = Claw.CLAWPOS.OPEN;
        robot.sleep(0.2);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.sleep(0.1);

        // Return back to 'scoreBasket1'
        robot.drive.setTargetPosition(scoreBasket1,true,true,0.7);
        waitUntilIdleDrive();
    }
    
    public void firstBucket() {
        // Lower arm for intaking
        robot.arm.setAutoTargetState(ArmState.INTAKING);
        waitUntilIdleArm();

        // Single move to the first bucket location
        robot.drive.setTargetPosition(bucketFirst,true,true,0.7);
        waitUntilIdleDrive();

        // Extend for the bucket
        robot.arm.setAutoTargetState(ArmState.INTAKING);
        robot.arm.changeExtension(firstBucketExtension);
        waitUntilIdleArm();

        // Grab
        robot.sleep(0.3);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.1);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;

        // Move back to 'scoreBasket1'
        robot.drive.setTargetPosition(scoreBasket1,true,true,0.7);
        robot.arm.setAutoTargetState(ArmState.HIGHBASKET);
        robot.arm.changeExtension(basketLength);
        waitUntilIdleArmAndDrive();

        // Now go to 'scoreBasket2' 
        robot.drive.setTargetPosition(scoreBasket2,true,true,0.7);
        waitUntilIdleDrive();

        // Drop
        robot.sleep(0.1);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.2);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.sleep(0.1);

        // Back again to 'scoreBasket1'
        robot.drive.setTargetPosition(scoreBasket1,true,true,0.7);
        waitUntilIdleDrive();
    }

    public void secondBucket() {
        // Similar flow to firstBucket, but going to 'bucketSecond'
        robot.arm.setAutoTargetState(ArmState.INTAKING);
        waitUntilIdleArm();

        robot.drive.setTargetPosition(bucketSecond,true,true,0.7);
        waitUntilIdleDrive();

        robot.arm.setAutoTargetState(ArmState.INTAKING);
        robot.arm.changeExtension(secondBucketExtension);
        waitUntilIdleArm();

        robot.sleep(0.3);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.1);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;

        robot.drive.setTargetPosition(scoreBasket1,true,true,0.7);
        robot.arm.setAutoTargetState(ArmState.HIGHBASKET);
        robot.arm.changeExtension(basketLength);
        waitUntilIdleArmAndDrive();

        robot.sleep(0.1);
        robot.drive.setTargetPosition(scoreBasket2,true,true,0.7);
        waitUntilIdleDrive();

        robot.sleep(0.1);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.2);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.sleep(0.1);

        robot.drive.setTargetPosition(scoreBasket1,true,true,0.7);
        waitUntilIdleDrive();
    }

    public void thirdBasket() {
        robot.arm.setAutoTargetState(ArmState.INTAKING);
        waitUntilIdleArm();

        robot.drive.setTargetPosition(bucketThird,true,true,0.7);
        waitUntilIdleDrive();

        robot.arm.setAutoTargetState(ArmState.INTAKING);
        robot.arm.changeExtension(thirdBucketExtension);
        waitUntilIdleArm();

        robot.sleep(0.3);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.DOWN;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
        robot.sleep(0.1);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;

        robot.drive.setTargetPosition(scoreBasket1,true,true,0.7);
        robot.arm.setAutoTargetState(ArmState.HIGHBASKET);
        robot.arm.changeExtension(basketLength);
        waitUntilIdleArmAndDrive();

        robot.sleep(0.1);
        robot.drive.setTargetPosition(scoreBasket2,true,true,0.7);
        waitUntilIdleDrive();

        robot.sleep(0.1);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.UP;
        robot.sleep(0.25);
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        robot.sleep(0.2);
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.sleep(0.1);

        robot.drive.setTargetPosition(scoreBasket1,true,true,0.7);
        waitUntilIdleDrive();
    }


    void park() {
        // Lower the arm
        robot.arm.setAutoTargetState(ArmState.INTAKING);
        waitUntilIdleArm();

        // Make sure tilt is mid for safe driving
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.sleep(0.1);

        // We pass the multi-waypoint path here
        robot.drive.setTargetPosition(parkPath,true,true,0.7);
        robot.arm.setAutoTargetState(ArmState.INTAKING);
        waitUntilIdleArmAndDrive();
    }

    ////////// Utility Wait Methods //////////

    private void waitUntilIdleDrive() {
        while(robot.drive.state != DriveTrain.STATE.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetryOutput();
            robot.sleep(0.01);
        }
    }

    private void waitUntilIdleArm() {
        while(robot.arm.currentState != Arm.FSMState.IDLE && opModeIsActive() && !isStopRequested()) {
            telemetryOutput();
            robot.sleep(0.01);
        }
    }

    private void waitUntilIdleArmAndDrive() {
        while((robot.drive.state != DriveTrain.STATE.IDLE
                || robot.arm.currentState != Arm.FSMState.IDLE)
                && opModeIsActive() && !isStopRequested()) {
            telemetryOutput();
            robot.sleep(0.01);
        }
    }

    private void telemetryOutput() {
        telemetry.addData("extension", robot.arm.extensionSubsystem.currentPos);
        telemetry.addData("robot arm", robot.arm.currentState);
        telemetry.addData("robot dt",  robot.drive.state);
        telemetry.addData("pose",      robot.drive.getPose());
        telemetry.update();
    }
}
