package org.firstinspires.ftc.teamcode.subsystems.Arm;


import static org.firstinspires.ftc.teamcode.subsystems.Arm.Pitch.PitchConstants.reverseSafePower;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.DEFAUlT;
import org.firstinspires.ftc.teamcode.Utils.ArmStates.STATE;
import org.firstinspires.ftc.teamcode.Utils.Globals;
import org.firstinspires.ftc.teamcode.Utils.Subsystem;
import org.firstinspires.ftc.teamcode.Utils.Wrappers.GamePadController;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Extension.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Extension.ExtensionConstants;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Pitch.Pitch;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Pitch.PitchConstants;

import java.util.LinkedList;
import java.util.Queue;

@Config
public class Arm implements Subsystem {
    public  boolean IS_DISABLED = false;
    public enum FSMState {
        EXTEND_MAX_FOR_TIME,
        IDLE,
        RETRACTING_EXTENSION,
        RESET_PITCH,
        ADJUSTING_PITCH,
        PRE_ADJUSTING_PITCH,
        EXTENDING_EXTENSION,
        MANUAL_CONTROL,
        OPERATION_COMPLETE,
        PRE_EXTENSION,
        CHANGING_OUTTAKE,
        EXTEND_TILL_SENSOR
    }

    public FSMState currentState = FSMState.IDLE;
    public ArmState targetState = ArmState.DEFAULT, previousState;
    public String TAG = "Robot";


    public boolean manualControl = false;
    public Extension extensionSubsystem;
    public Queue<FSMState> transitionPlan;
    public Pitch pitchSubsystem;
    public static double timerthreeshold = 120;
    public static double timerthreeshold2 = 15;

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
            extensionSubsystem = new Extension(hardwareMap, Globals.IS_AUTO, robot);
            Log.w("ARM", "Extenstion intialized successfully");
        } catch (Exception e) {
            Log.w(TAG, "Failed to initialize Extension: " + e.getMessage());
        }
        try {
            this.pitchSubsystem = new Pitch(hardwareMap, Globals.IS_AUTO,robot );
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
        planResetEncoders();
        adjustingPitchTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    }

    public static double raw_power_0 = 0.8;
    public static double raw_power_90 = 0.8;
    public boolean useRetractAuto = true;
    public double desiredExtension = 0;
    public double desiredPitch = 0;
    public static double predictValue = 80;

    public void setTargetState(ArmState newState) {
        Log.w("debug", "set target state");
        previousState = targetState;
        targetState = newState;
        desiredExtension = targetState.getExtensionTarget();
        desiredPitch = targetState.getPivotAngle();
        planTransitionSteps();
        currentState = nextStateInPlan();
    }

    public void changeExtension(double extension) {
        transitionPlan.clear();
        transitionPlan.add(FSMState.PRE_EXTENSION);
        transitionPlan.add(FSMState.EXTENDING_EXTENSION);
        desiredExtension = extension;
        desiredPitch = targetState.getPivotAngle();
        currentState = nextStateInPlan();
    }
    public void changePitch(double pitch) {
        transitionPlan.clear();

        transitionPlan.add(FSMState.ADJUSTING_PITCH);
        adjustingPitchTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        desiredPitch = pitch;
        pitchSubsystem.setTarget(desiredPitch);
        currentState = nextStateInPlan();
    }

    public void changeDesiredExtension(double extension) {
        desiredExtension = extension;
    }
    public void setAutoTargetState(ArmState newState) {
        previousState = targetState;
        targetState = newState;
        planAutoTransitionSteps();
        desiredExtension = targetState.getExtensionTarget();
        desiredPitch= targetState.getPivotAngle();
        currentState = nextStateInPlan();
    }
    public void fakePid(double addon) {
        transitionPlan.clear();
        desiredExtension = extensionSubsystem.currentPos + addon;
        maxTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        transitionPlan.add(FSMState.EXTEND_MAX_FOR_TIME);
        currentState = nextStateInPlan();
    }
    ElapsedTime partTimer = null;

    public static double timer0 = 20;

    ElapsedTime maxTimer = null;
    ElapsedTime atThreeshold = null;
    ElapsedTime timerExtension = null;
    ElapsedTime claW = null;
    ElapsedTime adjustingPitchTimer = null;
    public void update() {

        Log.w("Debug","arm case " + currentState);
        Log.w("Debug","arm desiredEx" + desiredExtension);
        if(IS_DISABLED)  {
            return;
        }
//        if (currentTime - lastUpdateTime < UPDATE_INTERVAL_MS) {
//            return;
//        }
        if (!manualControl && extensionSubsystem.mode!= Extension.MODE.RAW_POWER && currentState != FSMState.RETRACTING_EXTENSION && currentState!=FSMState.EXTEND_MAX_FOR_TIME  && currentState != FSMState.EXTENDING_EXTENSION) {
            extensionSubsystem.mode = Extension.MODE.IDLE;
        }
        switch (currentState) {
            case IDLE:
                if (desiredPitch == ExtensionConstants.at0positionPitch) pitchSubsystem.mode = Pitch.MODE.IDLE;
                claW = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                break;
            case CHANGING_OUTTAKE:
                Log.w("debug", "update");
                if(targetState == ArmState.DEFAULT || targetState == ArmState.SPECIMENGARD) clawSubsystem.clawPos = Claw.CLAWPOS.OPEN;
//                if((previousState == ArmState.INTAKING || previousState == ArmState.DEFAULT) && targetState == ArmState.HIGHBASKET) {
//                    clawSubsystem.clawPos = Claw.CLAWPOS.CLOSE;
//                    if(claW.time() > 20) {
//                        clawSubsystem.tiltState = targetState.getTiltState();
//                        clawSubsystem.rotateState = targetState.getRotatePos();
//                    }

                    clawSubsystem.tiltState = targetState.getTiltState();
                    clawSubsystem.rotateState = targetState.getRotatePos();

                currentState = nextStateInPlan();
                break;
            case RETRACTING_EXTENSION:
                extensionSubsystem.mode = Extension.MODE.RAW_POWER;
                extensionSubsystem.changeRawPower(-raw_power_0);
                if (extensionSubsystem.checkSwitch() == true) {
                    extensionSubsystem.mode = Extension.MODE.IDLE;
                    partTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                    currentState = nextStateInPlan();
                }
                break;
            case EXTEND_MAX_FOR_TIME:
                extensionSubsystem.mode = Extension.MODE.RAW_POWER;
                extensionSubsystem.changeRawPower(1);
                if(extensionSubsystem.currentPos > desiredExtension || maxTimer.time() > 2000) {
                    extensionSubsystem.mode = Extension.MODE.IDLE;
                    currentState = nextStateInPlan();
                }
                break;
            case RESET_PITCH:
                pitchSubsystem.mode = Pitch.MODE.RAW_POWER;
                pitchSubsystem.changeRawPower(reverseSafePower);
                if(pitchSubsystem.checkForSwitch()) {
                    pitchSubsystem.target = ExtensionConstants.at0positionPitch;
                    pitchSubsystem.mode = Pitch.MODE.IDLE;
                    currentState = nextStateInPlan();
                }
                break;
            case PRE_ADJUSTING_PITCH:

                if(desiredPitch == 0 && pitchSubsystem.getCurrentPos() != 0) {
                    desiredPitch = ExtensionConstants.at0positionPitch;
                }
                pitchSubsystem.setTarget(desiredPitch);
                adjustingPitchTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                currentState = nextStateInPlan();
                break;

            case ADJUSTING_PITCH:
                if(adjustingPitchTimer.time()>=2500) {
                    pitchSubsystem.mode = Pitch.MODE.IDLE;
                    resetMode();
                    break;
                }
                if ((desiredPitch!= ExtensionConstants.at0positionPitch && pitchSubsystem.isAtPosition(desiredPitch) || (desiredPitch == ExtensionConstants.at0positionPitch && pitchSubsystem.isAt0()))) {
                    if(atThreeshold == null) atThreeshold = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                    //extensionSubsystem.updateKerem(pitchSubsystem.calculateAngle());
                    // pitchSubsystem.updateRegreesion();
                    currentState = nextStateInPlan();
                }else {
                    atThreeshold = null;
                }
                break;
            case PRE_EXTENSION:
//                if(partTimer.time()>=timerthreeshold + 75) {
//                    extensionSubsystem.offset += extensionSubsystem.currentPos;
//
//                }
                timerExtension = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                extensionSubsystem.setTaget(desiredExtension);
                currentState = nextStateInPlan();
                break;
            case EXTENDING_EXTENSION:
//                if(timerExtension.time() > 2500) {
//                    transitionPlan.add(FSMState.RETRACTING_EXTENSION);
//                    currentState = nextStateInPlan();
//                }

                //                if (desiredExtension == 0) {
//                    currentState = nextStateInPlan();
//                    break;
//                }
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


        if(pitchSubsystem.calculateAngle() == 0 && extensionInput>=0) {
            if (Math.abs(lengthLimitAt0 - extensionSubsystem.currentPos) < extensionInput * predictValue) {
                extensionInput = 0;
            }
        }

        if(currentState != FSMState.IDLE) {
            if(targetState == ArmState.HIGHBASKET) {
                if(extensionInput < 0) extensionInput = 0;
            }else if(targetState == ArmState.INTAKING) {
                if(extensionInput > 0) {
                    extensionInput = 0;
                }
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
        if (gg.rightBumperOnce()) {
            if (clawSubsystem.tiltState == Claw.tiltMode.DOWN) {
                clawSubsystem.rotateState = Claw.RotateMode.ORIZONTAL;
                clawSubsystem.tiltState = Claw.tiltMode.MID;
            } else if (clawSubsystem.tiltState == Claw.tiltMode.MID) {
                clawSubsystem.tiltState = Claw.tiltMode.UP2;
            } else if (clawSubsystem.tiltState == Claw.tiltMode.UP2) {
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
                clawSubsystem.tiltState = Claw.tiltMode.UP2;
            } else if (clawSubsystem.tiltState == Claw.tiltMode.UP2) {
                clawSubsystem.tiltState = Claw.tiltMode.MID;
            }
        }
    }


    void planResetEncoders() {
        transitionPlan.clear();
        transitionPlan.add(FSMState.RETRACTING_EXTENSION);
        transitionPlan.add(FSMState.RESET_PITCH);
        currentState = nextStateInPlan();

    }



    private void planAutoTransitionSteps() {
        Log.w("debug", "auto transitions");

        transitionPlan.clear();
        transitionPlan.add(FSMState.CHANGING_OUTTAKE);
        transitionPlan.add(FSMState.RETRACTING_EXTENSION);

        transitionPlan.add(FSMState.PRE_ADJUSTING_PITCH);
        transitionPlan.add(FSMState.ADJUSTING_PITCH);
        //transitionPlan.add(FSMState.RETRACTING_EXTENSION);

        transitionPlan.add(FSMState.PRE_EXTENSION);
        transitionPlan.add(FSMState.EXTENDING_EXTENSION);

        transitionPlan.add(FSMState.OPERATION_COMPLETE);

    }

    private void planTransitionSteps() {
        Log.w("debug", "planning transition steps");
        transitionPlan.clear();
        transitionPlan.add(FSMState.CHANGING_OUTTAKE);
        if (useRetractAuto) transitionPlan.add(FSMState.RETRACTING_EXTENSION);

        transitionPlan.add(FSMState.PRE_ADJUSTING_PITCH);
        transitionPlan.add(FSMState.ADJUSTING_PITCH);
        //transitionPlan.add(FSMState.RETRACTING_EXTENSION);
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