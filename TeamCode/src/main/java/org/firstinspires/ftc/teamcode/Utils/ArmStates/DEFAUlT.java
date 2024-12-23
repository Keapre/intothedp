package org.firstinspires.ftc.teamcode.Utils.ArmStates;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw;

@Config
public class DEFAUlT extends STATE {
    public DEFAUlT() {
        this.clawpos = Claw.CLAWPOS.OPEN;
        this.rotatePos = Claw.RotateMode.ORIZONTAL;
        this.tiltState = Claw.tiltMode.DOWN;
        this.extensionTarget = 0; // Needs to be determined experimentally
        this.pitchAngle =0;
    }


}
