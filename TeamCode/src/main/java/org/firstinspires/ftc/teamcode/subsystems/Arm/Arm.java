package org.firstinspires.ftc.teamcode.subsystems.Arm;


import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.DEFAUlT;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.STATE;
import org.firstinspires.ftc.teamcode.Utils.Globals;
import org.firstinspires.ftc.teamcode.Utils.Subsystem;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Extension.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Pitch.Pitch;

import java.util.LinkedList;
import java.util.Queue;

@Config
public class Arm implements Subsystem {
    public static boolean IS_DISABLED = false;
    public enum FSMState {
        IDLE, RETRACTING_EXTENSION, ADJUSTING_PITCH, PRE_ADJUSTING_PITCH, EXTENDING_EXTENSION, MANUAL_CONTROL, OPERATION_COMPLETE,CHANGING_OUTTAKE
    }

    public FSMState currentState = FSMState.IDLE;
    public ArmState targetState, previousState;
    public String TAG = "Robot";


    public boolean manualControl = false;
    public Extension extensionSubsystem;
    public Queue<FSMState> transitionPlan;
    public Pitch pitchSubsystem;
    public static double timerthreeshold = 100;

    public Claw clawSubsystem;
    GamePadController gg;
    private long lastUpdateTime;
    double extensionInput = 0;
    private static final long UPDATE_INTERVAL_MS = 15;
    public static double lengthLimitAt0 = 600;
    Robot robot;

    public Arm(HardwareMap hardwareMap, boolean isAuto, Robot robot) {
        this.robot = robot;
        try {
            extensionSubsystem = new Extension(hardwareMap, Globals.isAuto(), robot);
            Log.w("ARM", "Extenstion intialized successfully");
        } catch (Exception e) {
            Log.w(TAG, "Failed to initialize Extension: " + e.getMessage());
        }
        try {
            this.pitchSubsystem = new Pitch(hardwareMap, Globals.isAuto(),robot );
            Log.w(TAG, "Pitch intialized successfully");
        } catch (Exception e) {
            Log.w(TAG, "Failed to initialize Pitch: " + e.getMessage());
        }
        try {
            this.clawSubsystem = new Claw(hardwareMap,robot);
            Log.w(TAG, "Claw intialized successfully");
        } catch (Exception e) {
            Log.w(TAG, "Failed to initialize Claw: " + e.getMessage());
        }
        transitionPlan = new LinkedList<>();
        targetState = ArmState.DEFAULT;
        previousState = targetState;
        currentState = FSMState.IDLE;

    }

    public static double raw_power_0 = 0.85;
    public static double raw_power_90 = 0.55;
    public boolean useRetractAuto = true;
    public double desiredExtension = 0;

    public void setTargetState(ArmState newState) {
        previousState = targetState;
        targetState = newState;
        desiredExtension = targetState.getExtensionTarget();

        planTransitionSteps();
        currentState = nextStateInPlan();
    }

    public void changeExtension(double extension) {
        transitionPlan.clear();
        transitionPlan.add(FSMState.EXTENDING_EXTENSION);
        desiredExtension = extension;
        currentState = nextStateInPlan();
    }

    public void setAutoTargetState(ArmState newState) {
        previousState = targetState;
        targetState = newState;
        planAutoTransitionSteps();
        desiredExtension = targetState.getExtensionTarget();
        currentState = nextStateInPlan();
    }


    public void changeDesiredExtension(double extension) {
        desiredExtension = extension;
    }

    public void update() {
        if(IS_DISABLED)  {
            return;
        }
//        if (currentTime - lastUpdateTime < UPDATE_INTERVAL_MS) {
//            return;
//        }
        if (!manualControl && currentState != FSMState.RETRACTING_EXTENSION && currentState != FSMState.EXTENDING_EXTENSION) {
            extensionSubsystem.mode = Extension.MODE.IDLE;
        } else if (!manualControl && currentState == FSMState.RETRACTING_EXTENSION) {
            extensionSubsystem.mode = Extension.MODE.AUTO;
        }
        switch (currentState) {
            case IDLE:
                if (targetState.getPivotAngle() == 0) pitchSubsystem.mode = Pitch.MODE.IDLE;
                break;
            case CHANGING_OUTTAKE:
                clawSubsystem.tiltState = targetState.getTiltState();
                clawSubsystem.rotateState = targetState.getRotatePos();
                break;
            case RETRACTING_EXTENSION:
                extensionSubsystem.mode = Extension.MODE.RAW_POWER;
                extensionSubsystem.changeRawPower(raw_power_0);
                if (pitchSubsystem.calculateAngle() > 80) {
                    extensionSubsystem.changeRawPower(raw_power_90);
                }
                if (extensionSubsystem.getTimer() >= timerthreeshold) {
                    extensionSubsystem.mode = Extension.MODE.IDLE;
                    currentState = nextStateInPlan();
                }
                break;
            case PRE_ADJUSTING_PITCH:
                extensionSubsystem.offset += extensionSubsystem.currentPos;
                pitchSubsystem.setMode(Pitch.MODE.AUTO);
                pitchSubsystem.setTarget(targetState.getPivotAngle());
                currentState = nextStateInPlan();
                break;

            case ADJUSTING_PITCH:
                if (pitchSubsystem.isAtPosition(targetState.getPivotAngle())) {
                    //extensionSubsystem.updateKerem(pitchSubsystem.calculateAngle());
                    if (targetState.getPivotAngle() == 0) pitchSubsystem.setMode(Pitch.MODE.IDLE);
                    currentState = nextStateInPlan();
                }
                break;

            case EXTENDING_EXTENSION:
                extensionSubsystem.mode = Extension.MODE.AUTO;
                if (desiredExtension == 0) {
                    currentState = nextStateInPlan();
                    break;
                }
                extensionSubsystem.setTaget(extensionSubsystem.offset + desiredExtension);
                if (extensionSubsystem.isAtPosition()) {
                    currentState = nextStateInPlan();
                }
                break;

            case OPERATION_COMPLETE:
                currentState = FSMState.IDLE;
                break;
        }
        pitchSubsystem.update();
        extensionSubsystem.update();
        clawSubsystem.update();
    }


    private void resetMode() {
        transitionPlan.clear();
        currentState = FSMState.IDLE;
        extensionSubsystem.mode = Extension.MODE.IDLE;
        pitchSubsystem.mode = Pitch.MODE.IDLE;

    }

    private void setPowerManual() {
        extensionInput = extensionInput * extensionInput * Math.signum(extensionInput);
        extensionSubsystem.manualControl(extensionInput);
    }

    public void handleManualControl(GamePadController gg) {
        extensionInput = -gg.left_trigger + gg.right_trigger;


        if(pitchSubsystem.calculateAngle() == 0 && extensionInput>0) {
            if (Math.abs((extensionSubsystem.offset + lengthLimitAt0) - extensionSubsystem.currentPos) < 40) {
                extensionInput = 0;
            }
        }



        if (Math.abs(gg.left_trigger) > 0.1 || Math.abs(gg.right_trigger) > 0.1) {
            setPowerManual();
        }

        if (gg.backOnce()) {

            useRetractAuto = !useRetractAuto;
        }
        if (gg.startOnce()) {
            extensionSubsystem.offset += extensionSubsystem.currentPos;
        }
        if (gg.aOnce()) {
            if (clawSubsystem.clawPos == Claw.CLAWPOS.OPEN)
                clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
            else clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
        }
//        if(gg.dpadLeftOnce()) {
//            if(clawSubsystem.rotateState == Claw.RotateMode.VERTICAL) {
//                clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
//            }else if(clawSubsystem.rotateState == Claw.RotateMode.ORIZONTAL) {
//                clawSubsystem.rotateState = Claw.RotateMode.VERTICAL;
//            }
//        }
//        if(gg.dpadRightOnce()) {
//            if(clawSubsystem.rotateState == Claw.RotateMode.VERTICAL) {
//                clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
//            }else if(clawSubsystem.rotateState == Claw.RotateMode.ORIZONTAL) {
//                clawSubsystem.rotateState = Claw.RotateMode.VERTICAL;
//            }
//        }
//        if(gg.dpadUpOnce()) {
//            if (clawSubsystem.tiltState == Claw.tiltMode.DOWN) {
//                clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
//                clawSubsystem.tiltState = Claw.tiltMode.MID;
//            } else if (clawSubsystem.tiltState == Claw.tiltMode.MID) {
//                clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
//                clawSubsystem.tiltState = Claw.tiltMode.UP;
//            }
//        }
//        if(gg.dpadDownOnce()) {
//            if(clawSubsystem.tiltState == Claw.tiltMode.UP) {
//                clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
//                clawSubsystem.tiltState = Claw.tiltMode.MID;
//            }else if(clawSubsystem.tiltState == Claw.tiltMode.MID) {
//                clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
//                clawSubsystem.tiltState = Claw.tiltMode.DOWN;
//            }
//        }
        if (gg.rightBumperOnce()) {
            if (clawSubsystem.tiltState == Claw.tiltMode.DOWN) {
                clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
                clawSubsystem.tiltState = Claw.tiltMode.MID;
            } else if (clawSubsystem.tiltState == Claw.tiltMode.MID) {
                clawSubsystem.tiltState = Claw.tiltMode.UP;
            }
        }

        if (gg.guideOnce()) {
            resetMode();
        }
        if (gg.leftBumperOnce()) {
            if (clawSubsystem.tiltState == Claw.tiltMode.DOWN && clawSubsystem.rotateState == Claw.RotateMode.VERTICAL) {
                clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
            } else if (clawSubsystem.tiltState == Claw.tiltMode.DOWN && clawSubsystem.rotateState == Claw.RotateMode.ORIZONTAL) {
                clawSubsystem.rotateState = Claw.RotateMode.VERTICAL;
            } else if (clawSubsystem.tiltState == Claw.tiltMode.MID) {
                clawSubsystem.tiltState = Claw.tiltMode.DOWN;
            } else if (clawSubsystem.tiltState == Claw.tiltMode.UP) {
                clawSubsystem.tiltState = Claw.tiltMode.MID;
            }
        }
    }



    private void planAutoTransitionSteps() {
        transitionPlan.clear();
        transitionPlan.add(FSMState.CHANGING_OUTTAKE);
        transitionPlan.add(FSMState.RETRACTING_EXTENSION);

        transitionPlan.add(FSMState.PRE_ADJUSTING_PITCH);
        transitionPlan.add(FSMState.ADJUSTING_PITCH);

        transitionPlan.add(FSMState.EXTENDING_EXTENSION);

        transitionPlan.add(FSMState.OPERATION_COMPLETE);

    }

    private void planTransitionSteps() {
        transitionPlan.clear();
        transitionPlan.add(FSMState.CHANGING_OUTTAKE);
        if (useRetractAuto) transitionPlan.add(FSMState.RETRACTING_EXTENSION);

        transitionPlan.add(FSMState.PRE_ADJUSTING_PITCH);
        transitionPlan.add(FSMState.ADJUSTING_PITCH);

//        if (Math.abs(extensionSubsystem.getCurrentPosition() - targetState.extensionTarget) > 10) {
//            transitionPlan.add(FSMState.EXTENDING_EXTENSION);
//        }

        transitionPlan.add(FSMState.OPERATION_COMPLETE);

    }

    private FSMState nextStateInPlan() {
        FSMState peek = transitionPlan.peek();
        if (peek == null) return FSMState.IDLE;
        transitionPlan.remove();
        return peek;
    }

    public FSMState getCurrentState() {
        return currentState;
    }
}