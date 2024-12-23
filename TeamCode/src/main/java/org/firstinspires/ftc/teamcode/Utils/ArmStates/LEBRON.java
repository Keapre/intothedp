package org.firstinspires.ftc.teamcode.Utils.ArmStates;

import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw;

public class LEBRON extends STATE {
    public LEBRON() {
        this.clawpos = Claw.CLAWPOS.CLOSE;
        this.rotatePos = Claw.RotateMode.ORIZONTAL;
        this.tiltState = Claw.tiltMode.MID;
        this.extensionTarget = 0; // Needs to be determined experimentally
        this.pitchAngle = 580;
    }
}
