package org.firstinspires.ftc.teamcode.Utils.ArmStates;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Pitch;

@Config
public class SPECIMEN extends STATE{
    public SPECIMEN() {
        this.clawpos = Claw.CLAWPOS.CLOSE;
        this.rotatePos = Claw.ROTATESTATE.DEFAULT;
        this.tiltState = Claw.TILTSTATE.MID;
        this.extensionTarget = 600; // Needs to be determined experimentally
        this.pitchAngle = Pitch.PITCHPOS.HIGH_CHAMBER;
    }

}
