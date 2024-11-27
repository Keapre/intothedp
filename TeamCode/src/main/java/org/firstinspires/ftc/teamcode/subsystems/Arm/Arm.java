package org.firstinspires.ftc.teamcode.subsystems.Arm;


import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.ArmStates.DEFAUlT;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.STATE;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.Utils.pubSub.Subsystem;
import org.firstinspires.ftc.teamcode.opmode.tests.PitchTest;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain.MecanumTest;

import java.util.LinkedList;
import java.util.Queue;

public class Arm implements Subsystem {
    public  enum FSMState {
        IDLE,
        RETRACTING_EXTENSION,
        ADJUSTING_PITCH,
        PRE_ADJUSTING_PITCH,
        EXTENDING_EXTENSION,
        MANUAL_CONTROL,
        OPERATION_COMPLETE
    }

    public FSMState currentState = FSMState.IDLE;
    public STATE targetState;
    public String TAG = "Robot";


    public boolean manualControl = false;
    public Extension extensionSubsystem;
    public Queue<FSMState> transitionPlan;
    public Pitch pitchSubsystem;
    public Claw clawSubsystem;
    GamePadController gg;
    private long lastUpdateTime;
    double extensionInput = 0;
    private static final long UPDATE_INTERVAL_MS = 15;

    public Arm(HardwareMap hardwareMap,boolean isAuto) {
        try {
            extensionSubsystem = new Extension(hardwareMap,false,this);
            Log.w("ARM", "Extenstion intialized successfully");
        } catch (Exception e) {
            Log.w(TAG, "Failed to initialize Extension: " + e.getMessage());
        }
        try {
            this.pitchSubsystem = new Pitch(hardwareMap,false,this);
            Log.w(TAG, "Pitch intialized successfully");
        } catch (Exception e) {
            Log.w(TAG, "Failed to initialize Pitch: " + e.getMessage());
        }
        try {
            this.clawSubsystem = new Claw(hardwareMap,false);
            Log.w(TAG, "Claw intialized successfully");
        } catch (Exception e) {
            Log.w(TAG, "Failed to initialize Claw: " + e.getMessage());
        }
        transitionPlan = new LinkedList<>();
        setTargetState(new DEFAUlT());

        currentState = FSMState.IDLE;

    }

    public void setTargetState(STATE newState) {
        targetState = newState;
        planTransitionSteps();
        currentState = nextStateInPlan();
    }
    private void clearQueue() {
        while(!transitionPlan.isEmpty()) {
            transitionPlan.remove();
        }
    }
    public void update() {
        long currentTime = System.currentTimeMillis();
//        if (currentTime - lastUpdateTime < UPDATE_INTERVAL_MS) {
//            return;
//        }
        lastUpdateTime = currentTime;


        if (manualControl) {
            currentState = FSMState.MANUAL_CONTROL;
            extensionSubsystem.mode = Extension.MODE.MANUAL;
            pitchSubsystem.isMotionProfileActive = false;
        }

        switch (currentState) {
            case IDLE:
                pitchSubsystem.mode = Pitch.MODE.IDLE;
                extensionSubsystem.mode = Extension.MODE.IDLE;
                break;

            case MANUAL_CONTROL:
                setPowerManual();
                if (!manualControl) {
                    pitchSubsystem.setMode(Pitch.MODE.IDLE);
                    extensionSubsystem.mode = Extension.MODE.AUTO;
                    currentState = FSMState.IDLE;
                }
                break;

            case RETRACTING_EXTENSION:
                extensionSubsystem.mode = Extension.MODE.AUTO;
                extensionSubsystem.target = extensionSubsystem.offset + 5;
                if (extensionSubsystem.isAtZero()) {
                    currentState = nextStateInPlan();
                }
                break;
            case PRE_ADJUSTING_PITCH:
                pitchSubsystem.setTarget(targetState.getPitchAngle());
                pitchSubsystem.setMode(Pitch.MODE.AUTO);
                pitchSubsystem.startMotionProfile(targetState.getPitchAngle());
                currentState = nextStateInPlan();
                break;

            case ADJUSTING_PITCH:
                pitchSubsystem.setMode(Pitch.MODE.AUTO);
                if (!pitchSubsystem.isMotionProfileActive) {
                    pitchSubsystem.setMode(Pitch.MODE.IDLE);
                    currentState = nextStateInPlan();
                }

                break;

            case EXTENDING_EXTENSION:
                extensionSubsystem.mode = Extension.MODE.AUTO;
                extensionSubsystem.target =targetState.getExtensionTarget();
                if (extensionSubsystem.isAtPosition(targetState.getExtensionTarget())) {
                    currentState = nextStateInPlan();
                }
                break;

            case OPERATION_COMPLETE:
                clawSubsystem.clawPos =targetState.getClawpos();
                clawSubsystem.rotateState=targetState.getRotatePos();
                clawSubsystem.tiltState=targetState.getTilt();
                pitchSubsystem.mode= Pitch.MODE.IDLE;


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
        extensionInput = gg.left_trigger + -gg.right_trigger;

        if(gg.aOnce()) {
            if(clawSubsystem.clawPos == Claw.CLAWPOS.OPEN) clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
            else clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        }
        if(gg.dpadLeftOnce()) {
            if(clawSubsystem.rotateState == Claw.RotateMode.VERTICAL) {
                clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
            }else if(clawSubsystem.rotateState == Claw.RotateMode.ORIZONTAL) {
                clawSubsystem.rotateState = Claw.RotateMode.VERTICAL;
            }
        }
        if(gg.dpadRightOnce()) {
            if(clawSubsystem.rotateState == Claw.RotateMode.VERTICAL) {
                clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
            }else if(clawSubsystem.rotateState == Claw.RotateMode.ORIZONTAL) {
                clawSubsystem.rotateState = Claw.RotateMode.VERTICAL;
            }
        }
        if(gg.dpadUpOnce()) {
            if (clawSubsystem.tiltState == Claw.tiltMode.DOWN) {
                clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
                clawSubsystem.tiltState = Claw.tiltMode.MID;
            } else if (clawSubsystem.tiltState == Claw.tiltMode.MID) {
                clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
                clawSubsystem.tiltState = Claw.tiltMode.UP;
            }
        }
        if(gg.dpadDownOnce()) {
            if(clawSubsystem.tiltState == Claw.tiltMode.UP) {
                clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
                clawSubsystem.tiltState = Claw.tiltMode.MID;
            }else if(clawSubsystem.tiltState == Claw.tiltMode.MID) {
                clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
                clawSubsystem.tiltState = Claw.tiltMode.DOWN;
            }
        }
    }


    private void updateCurrentStateFromSensors() {

    }



    private void planTransitionSteps() {
        transitionPlan.clear();
//        if (extensionSubsystem.getCurrentPosition() > extensionSubsystem.offset + 10) {
//            transitionPlan.add(FSMState.RETRACTING_EXTENSION);
//        }

        transitionPlan.add(FSMState.PRE_ADJUSTING_PITCH);
        transitionPlan.add(FSMState.ADJUSTING_PITCH);

//        if (Math.abs(extensionSubsystem.getCurrentPosition() - targetState.extensionTarget) > 10) {
//            transitionPlan.add(FSMState.EXTENDING_EXTENSION);
//        }

        transitionPlan.add(FSMState.OPERATION_COMPLETE);

    }

    private FSMState nextStateInPlan() {
        FSMState peek = transitionPlan.peek();
        if(peek==null) return FSMState.IDLE;
        return transitionPlan.poll();
    }

    public FSMState getCurrentState() {
        return currentState;
    }
}
