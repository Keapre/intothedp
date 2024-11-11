package org.firstinspires.ftc.teamcode.Utils.ArmStates;

import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Pitch;

public class HIGHBASKET extends STATE {
    public HIGHBASKET() {
        this.clawpos = Claw.CLAWPOS.CLOSE;
        this.rotatePos = Claw.ROTATESTATE.DEFAULT;
        this.tiltState = Claw.TILTSTATE.UP;
        this.extensionTarget = 1200; // Needs to be determined experimentally
        this.pitchAngle = Pitch.PITCHPOS.HIGH_BASKET;
    }
}
