package org.firstinspires.ftc.teamcode.Utils.ArmStates;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Pitch;

@Config
public class INTAKING extends STATE{
    public INTAKING() {
        this.clawpos = Claw.CLAWPOS.OPEN;
        this.rotatePos = Claw.RotateMode.ORIZONTAL;
        this.tiltState = Claw.tiltMode.MID;
        this.extensionTarget = 0; // Needs to be determined experimentally
        this.pitchAngle = 0;
    }

}
