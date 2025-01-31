package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.Utils.OpModeUtils.setOpMode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.DEFAUlT;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.HIGHBASKET;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.INTAKING;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.SPECIMEN;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.SPECIMENGARD;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.SPECIMENSLAM;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.SPECIMENTELEOP;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.STATE;
import org.firstinspires.ftc.teamcode.Utils.Files.BlackBox.BlackBoxLogger;
import org.firstinspires.ftc.teamcode.Utils.Files.BlackBox.BlackBoxTestingOp;
import org.firstinspires.ftc.teamcode.Utils.Globals;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.TelemetryUtil;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Hang.Hang;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Blue")
public class TeleOp extends OpMode {
    Robot robot;
    GamePadController gg,gg2;
    Pose2d startPose =  new Pose2d(0,0,Math.toRadians(270));

    long lastLoopFinish;

    public static double slow_extension_limit = 200;
    ElapsedTime timer = null;
    BlackBoxLogger logger = null;
    private boolean endgame = false,park = false,park2 = false;
    public static double parkTime = 10;
    public static double parkTime2 = 5;
    public static boolean useLogger = false;

    @Override
    public void init() {
        TelemetryUtil.setup();
        Globals.IS_AUTO = false;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gg = new GamePadController(gamepad1);
        gg2 = new GamePadController(gamepad2);
        robot = new Robot(this,startPose);
        lastLoopFinish = System.currentTimeMillis();
    }


    @Override
    public void start() {
        robot.start();
        robot.arm.clawSubsystem.tiltState = Claw.tiltMode.MID;
        robot.arm.clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }
    private boolean isManualControlActivated() {
        return Math.abs(gg.left_trigger) > 0.1 || Math.abs(gg.right_trigger) > 0.1 || gg.aOnce() ||
                gg.dpadUpOnce() || gg.dpadDownOnce() || gg.dpadLeftOnce() || gg.dpadRightOnce() || gg.leftBumperOnce()
                || gg.rightBumperOnce() || gg.guideOnce() || gg.startOnce() || gg.backOnce();
    }

    private boolean manualControlDeactivated() {
        return Math.abs(gg.left_trigger) < 0.1 && Math.abs(gg.right_trigger) < 0.1 && !gg.aOnce() &&
                !gg.dpadUpOnce() && !gg.dpadDownOnce() && !gg.dpadLeftOnce() && !gg.dpadRightOnce()
                && !gg.leftBumperOnce() && !gg.rightBumperOnce() && !gg.guideOnce() && !gg.startOnce() && !gg.backOnce();
    }


    private static String formatResults(MovingStatistics statistics) {
        return Misc.formatInvariant("μ = %.2fms, σ = %.2fms, err = %.3fms",
                statistics.getMean() * 1000,
                statistics.getStandardDeviation() * 1000,
                statistics.getStandardDeviation() / Math.sqrt(statistics.getCount()) * 1000);
    }

    private void addStatistics() {
        telemetry.addData("Top 250", formatResults(robot.top250));
        telemetry.addData("Top 100", formatResults(robot.top100));
        telemetry.addData("Top 10", formatResults(robot.top10));
    }
    @Override
    public void loop()
    {
        gg2.update();
        gg.update();
        updateDrive();
        updateArm();
        hang();
        updateTelemetry();
        if(useLogger) logger.writeData();
    }

    public void updateArm() {
        if(isManualControlActivated()) {
            robot.arm.handleManualControl(gg);
            robot.arm.manualControl = true;
        }else if(manualControlDeactivated()) {
            robot.arm.manualControl = false;
        }
        if(gg.bOnce()) {
            Log.w("debug", "b pressed");

            if(robot.arm.targetState == ArmState.HIGHBASKET && robot.arm.getCurrentState() == Arm.FSMState.IDLE) {
                robot.arm.setTargetState(ArmState.DEFAULT);
            }
            else if(robot.arm.getCurrentState() == Arm.FSMState.IDLE) {
                robot.arm.setTargetState(ArmState.INTAKING);
            }
        }
        if (gg.yOnce()) {
            Log.w("debug", "y pressed");

//            if( robot.arm.getCurrentState() == Arm.FSMState.IDLE && robot.arm.targetState==ArmState.INTAKING) {
//                robot.arm.setTargetState(ArmState.SPECIMENGARD);
//            } else if(robot.arm.getCurrentState() == Arm.FSMState.IDLE && robot.arm.targetState==ArmState.SPECIMENGARD) {
//                robot.arm.setTargetState(ArmState.SpecimenBar);
//            }
            if(robot.arm.getCurrentState() == Arm.FSMState.IDLE)
            {
                switch (robot.arm.targetState)
                {
                    case SPECIMENGARD:
                        robot.arm.setTargetState(ArmState.SpecimenBar);
                        break;
                    case SpecimenBar:
                        robot.arm.setTargetState(ArmState.SPECIMENGARD);
                        break;
                    default:
                        robot.arm.setTargetState(ArmState.SPECIMENGARD);
                        break;
                }
            }

        }

        if(gg.xOnce()) {
            Log.w("debug", "x pressed");
            if( robot.arm.getCurrentState() == Arm.FSMState.IDLE) {
                robot.arm.setTargetState(ArmState.HighBasketTeleOp);
            }
        }
    }
    public void updateDrive() {
        //total = 2 min = 120 sec
        if(timer.time() > 90 && !endgame) {
            endgame = true;
            gg.rumble(450);
        }
        if(!park && timer.time() > 120 - parkTime) {
            park = true;
            gg.rumble(450);
        }
        if(!park2 && timer.time() > 120 - parkTime2) {
            park2 = true;
            gg.rumble(450);
        }
        robot.drive.slow_mode = robot.arm.extensionSubsystem.getPosition() >= slow_extension_limit;

//        if(robot.arm.targetState == ArmState.SPECIMENGARD) {
//            robot.drive.slow_mode = true;
//        }
        robot.drive.drive(gg);
    }

    public void hang() {
//        if(gg2.aOnce()) {
//            robot.hang.setServo();
//        }
//
//        if(gg2.bOnce()) {
//            robot.hang.setCloseServo();
//        }
//
//        if(gg2.yOnce()) {
//            robot.hang.startClimb();
//        }
//        robot.hang.changePower(gg2);
        if(gg.dpadLeftOnce()){
            robot.hang.setServo();
        }
        if(gg.dpadRightOnce()){
            robot.hang.setCloseServo();
        }
        if(gg.dpadUp()){
            robot.hang.changePower(-1);
        }
       else  if(gg.dpadDown()){
            robot.hang.changePower(1);
        }
       else{
           robot.hang.changePower(0);
        }
       if(gg.rightStickButtonOnce()){
           robot.hang.startClimb();
       }
    }
    public void updateTelemetry() {
        telemetry.addLine();
        telemetry.addLine("---------------ARM--------------");
        telemetry.addData("ARM STATE",robot.arm.targetState);
        telemetry.addData("ARM CURRENT STATE",robot.arm.getCurrentState());
        telemetry.addData("ARM MANUAL CONTROL",robot.arm.manualControl);
        telemetry.addLine();
        telemetry.addData("useRetract",robot.arm.useRetractAuto);
        telemetry.addData("ff",robot.arm.extensionSubsystem.ff);
        telemetry.addData("Extension power",robot.arm.extensionSubsystem.power);
        telemetry.addData("Extension pos",robot.arm.extensionSubsystem.currentPos);
        telemetry.addData("extension offset",robot.arm.extensionSubsystem.offset);
        telemetry.addData("maxx Amp",robot.arm.extensionSubsystem.getMaxAmps());
        telemetry.addData("Amp",robot.arm.extensionSubsystem.getAmp());
        telemetry.addData("Pith angle",robot.arm.pitchSubsystem.get_angle());
        telemetry.addData("Pith pos",robot.arm.pitchSubsystem.getCurrentPos());
        telemetry.addData("target angle",robot.arm.targetState.getPivotAngle());
        telemetry.addData("target pitch",robot.arm.pitchSubsystem.getTarget());
        telemetry.addData("pitch mode",robot.arm.pitchSubsystem.mode);

        telemetry.addData("extend mode",robot.arm.extensionSubsystem.mode);
        telemetry.addData("Claw pos",robot.arm.clawSubsystem.clawPos);
        telemetry.addData("Claw state",robot.arm.clawSubsystem.tiltState);
        telemetry.addData("pitch ff",robot.arm.pitchSubsystem.ff);
        telemetry.addData("extension switch",robot.arm.extensionSubsystem.checkSwitch());
        telemetry.addData("slowMode",robot.drive.slow_mode);
        telemetry.addLine();
        telemetry.addData("timer",timer.time());
        telemetry.addData("Sample Rate (Hz) ",1/((double)(System.currentTimeMillis() - lastLoopFinish)/1000.0));
        addStatistics();
        telemetry.update();
        TelemetryUtil.sendTelemetry();
        lastLoopFinish = System.currentTimeMillis();
    }

    @Override
    public void stop() {
        robot.stop();
    }

}