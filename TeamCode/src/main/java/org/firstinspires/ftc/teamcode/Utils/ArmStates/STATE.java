package org.firstinspires.ftc.teamcode.Utils.ArmStates;

import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw;

public class STATE {
    public  Claw.CLAWPOS clawpos;
    public  Claw.RotateMode rotatePos;

    public   Claw.tiltMode tiltState;

    public  double extensionTarget;
    public  double pitchAngle;


    public double getPitchAngle() {
        return pitchAngle;
    }
    public double getExtensionTarget() {
        return extensionTarget;
    }

    public Claw.tiltMode getTilt() {
        return tiltState;
    }
    public Claw.RotateMode getRotatePos() {
        return rotatePos;
    }

    public Claw.CLAWPOS getClawpos() {
        return clawpos;
    }

}
