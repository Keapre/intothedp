package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.DEFAUlT;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.HIGHBASKET;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.INTAKING;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.SPECIMEN;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.SPECIMENGARD;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.SPECIMENTELEOP;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.STATE;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Arm;



@TeleOp(name = "eric e smech")
public class EricTeleOp extends OpMode {
    Robot robot;
    GamePadController gg;

    double test = 0;
    long lastLoopFinish;

    STATE SpeciemnTeleOp = new SPECIMENTELEOP();
    STATE DEFAULT = new DEFAUlT();
    STATE HIGHBASKET = new HIGHBASKET();
    STATE INTAKING = new INTAKING();
    STATE specimenbar = new SPECIMEN();
    STATE specimengard = new SPECIMENGARD();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gg = new GamePadController(gamepad1);
        robot = new Robot(this,false);
        lastLoopFinish = System.currentTimeMillis();
    }


    @Override
    public void start() {
        robot.start();
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
        gg.update();
        updateDrive();
        updateArm();
        updateTelemetry();
    }

    public void updateArm() {
        if(isManualControlActivated()) {
            robot.arm.handleManualControl(gg);
            robot.arm.manualControl = true;
        }else if(manualControlDeactivated()) {
            robot.arm.manualControl = false;
        }
        if(gg.bOnce()) {
            if(robot.arm.getCurrentState() == Arm.FSMState.IDLE) {
                robot.arm.setTargetState(INTAKING);
            }
        }
        if (gg.yOnce()) {
            if( robot.arm.getCurrentState() == Arm.FSMState.IDLE && robot.arm.targetState == specimengard) {
                robot.arm.setTargetState(SpeciemnTeleOp);
            }else if(robot.arm.getCurrentState() == Arm.FSMState.IDLE && robot.arm.targetState != specimengard) {
                robot.arm.setTargetState(specimengard);
            }
        }
        if(gg.xOnce()) {
            if( robot.arm.getCurrentState() == Arm.FSMState.IDLE) {
                robot.arm.setTargetState(HIGHBASKET);
            }
        }
    }
    public void updateDrive() {
         robot.drive.setMotorPowersFromGamepad(gg,1.0,false,true);
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
        telemetry.addData("Pith angle",robot.arm.pitchSubsystem.calculateAngle());
        telemetry.addData("Pith pos",robot.arm.pitchSubsystem.getCurrentPos());
        telemetry.addData("target angle",robot.arm.targetState.getPitchAngle());
        telemetry.addData("target pitch",robot.arm.pitchSubsystem.target);
        telemetry.addData("pitch mode",robot.arm.pitchSubsystem.mode);

        telemetry.addData("timer",robot.arm.extensionSubsystem.getTimer());
        telemetry.addData("extend mode",robot.arm.extensionSubsystem.mode);
        telemetry.addData("Claw pos",robot.arm.clawSubsystem.clawPos);
        telemetry.addData("Claw state",robot.arm.clawSubsystem.tiltState);
        telemetry.addData("motion profile over",robot.arm.pitchSubsystem.isMotionProfileActive);
        telemetry.addData("vel",robot.arm.pitchSubsystem.desiredVelocity);
        telemetry.addData("isRetracted",robot.arm.extensionSubsystem.isRetracted());
        telemetry.addData("pitch ff",robot.arm.pitchSubsystem.ff);
        telemetry.addData("slowMode",robot.drive.slow_mode);
        telemetry.addLine();
        telemetry.addData("Sample Rate (Hz) ",1/((double)(System.currentTimeMillis() - lastLoopFinish)/1000.0));
        addStatistics();
        telemetry.update();
        lastLoopFinish = System.currentTimeMillis();
    }

    @Override
    public void stop() {
        robot.stop();
    }

}