package org.firstinspires.ftc.teamcode.subsystems.Arm;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.ArmStates.DEFAUlT;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.STATE;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.Utils.pubSub.Subsystem;

public class Arm implements Subsystem {
    public  enum FSMState {
        IDLE,
        RETRACTING_EXTENSION,
        ADJUSTING_PITCH,
        EXTENDING_EXTENSION,
        MANUAL_CONTROL,
        OPERATION_COMPLETE
    }

    public FSMState currentState = FSMState.IDLE;
    public STATE targetState;
    public boolean manualControl = false;
    public Extension extensionSubsystem;
    public Pitch pitchSubsystem;
    public Claw clawSubsystem;
    GamePadController gg;
    private long lastUpdateTime;
    double extensionInput = 0;
    private static final long UPDATE_INTERVAL_MS = 15;

    public Arm(HardwareMap hardwareMap,boolean isAuto,GamePadController gg) {
        this.extensionSubsystem = new Extension(hardwareMap,false,this);
        this.pitchSubsystem = new Pitch(hardwareMap,false);
        this.clawSubsystem = new Claw(hardwareMap,false);
        setTargetState(new DEFAUlT());
        this.gg = gg;
    }

    public void setTargetState(STATE newState) {
        this.targetState = newState;
        planTransitionSteps();
        currentState = nextStateInPlan();
    }
    public void update() {
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastUpdateTime < UPDATE_INTERVAL_MS) {
            return;
        }
        lastUpdateTime = currentTime;

        if (manualControl) {
            currentState = FSMState.MANUAL_CONTROL;
            pitchSubsystem.mode = Pitch.MODE.MANUAL;
            extensionSubsystem.mode = Extension.MODE.MANUAL;
        }

        switch (currentState) {
            case IDLE:
                break;

            case MANUAL_CONTROL:
                setPowerManual();
                if (!manualControl) {
                    pitchSubsystem.mode = Pitch.MODE.AUTO;
                    extensionSubsystem.mode = Extension.MODE.AUTO;
                    updateCurrentStateFromSensors();
                    currentState = FSMState.IDLE;
                }
                break;

            case RETRACTING_EXTENSION:
                extensionSubsystem.target = 3;
                if (extensionSubsystem.isAtZero()) {
                    currentState = nextStateInPlan();
                }
                break;

            case ADJUSTING_PITCH:
                pitchSubsystem.target = (targetState.pitchAngle);
                if (pitchSubsystem.isAtPosition(targetState.pitchAngle)) {
                    currentState = nextStateInPlan();
                }
                break;

            case EXTENDING_EXTENSION:
                extensionSubsystem.target =targetState.extensionTarget;
                if (extensionSubsystem.isAtPosition(targetState.extensionTarget)) {
                    currentState = nextStateInPlan();
                }
                break;

            case OPERATION_COMPLETE:
                clawSubsystem.clawPos =targetState.clawpos;
                clawSubsystem.rotateState=targetState.rotatePos;
                clawSubsystem.tiltState=targetState.tiltState;

                currentState = FSMState.IDLE;
                break;

            default:
                currentState = FSMState.IDLE;
                break;
        }
        pitchSubsystem.update();
        extensionSubsystem.update();
        clawSubsystem.update();
    }


    private void setPowerManual() {
        extensionSubsystem.manualControl(extensionInput);
    }
    public void handleManualControl(GamePadController gg) {
        double extensionInput = -gg.left_trigger + gg.right_trigger;

        if(gg.aOnce()) {
            if(clawSubsystem.clawPos == Claw.CLAWPOS.OPEN) clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
            else clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        }
        if(gg.rightBumper()) {
            clawSubsystem.rotateState = clawSubsystem.rotateState.next();
        }
        if(gg.leftBumper()) {
            clawSubsystem.rotateState = clawSubsystem.rotateState.previous();
        }
    }


    private void updateCurrentStateFromSensors() {

    }

    private java.util.Queue<FSMState> transitionPlan = new java.util.LinkedList<>();

    private void planTransitionSteps() {
        transitionPlan.clear();

        if (extensionSubsystem.getCurrentPosition() > extensionSubsystem.offset + 10) {
            transitionPlan.add(FSMState.RETRACTING_EXTENSION);
        }

        if (!pitchSubsystem.isAtPosition(targetState.pitchAngle)) {
            transitionPlan.add(FSMState.ADJUSTING_PITCH);
        }

        if (Math.abs(extensionSubsystem.getCurrentPosition() - targetState.extensionTarget) > 10) {
            transitionPlan.add(FSMState.EXTENDING_EXTENSION);
        }

        transitionPlan.add(FSMState.OPERATION_COMPLETE);
    }

    private FSMState nextStateInPlan() {
        return transitionPlan.poll() != null ? transitionPlan.poll() : FSMState.IDLE;
    }

    public FSMState getCurrentState() {
        return currentState;
    }
}
