package org.firstinspires.ftc.teamcode.Utils.ArmStates;

import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Pitch;

public class HIGHBASKET extends STATE {
    public HIGHBASKET() {
        this.clawpos = Claw.CLAWPOS.CLOSE;
        this.rotatePos = Claw.RotateMode.ORIZONTAL;
        this.tiltState = Claw.tiltMode.MID;
        this.extensionTarget = 0; // Needs to be determined experimentally
        this.pitchAngle = 555;
    }
}
