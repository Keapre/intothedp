package org.firstinspires.ftc.teamcode.Utils.ArmStates;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw.Claw;

@Config
public class SPECIMENTELEOP extends STATE{
    public SPECIMENTELEOP() {
        this.clawpos = Claw.CLAWPOS.CLOSE;
        this.rotatePos = Claw.RotateMode.ORIZONTAL;
        this.tiltState = Claw.tiltMode.MID;
        this.extensionTarget = 345; // Needs to be determined experimentally //si dupa scazi 125
        this.pitchAngle = 230;
    }

}
