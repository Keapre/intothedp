package org.firstinspires.ftc.teamcode.Utils.ArmStates;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw.Claw;

@Config
public class SPECIMEN extends STATE{
    public SPECIMEN() {
        this.clawpos = Claw.CLAWPOS.CLOSE;
        this.rotatePos = Claw.RotateMode.ORIZONTAL;
        this.tiltState = Claw.tiltMode.DOWN;
        this.extensionTarget = 430; // Needs to be determined experimentally //si dupa scazi 125
        this.pitchAngle = 320;
    }

}
