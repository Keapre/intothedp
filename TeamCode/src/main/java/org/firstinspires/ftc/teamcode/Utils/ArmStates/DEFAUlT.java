package org.firstinspires.ftc.teamcode.Utils.ArmStates;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Arm.Pitch;

@Config
public class DEFAUlT extends STATE {
    public DEFAUlT() {
        this.clawpos = Claw.CLAWPOS.CLOSE;
        this.rotatePos = Claw.ROTATESTATE.DEFAULT;
        this.tiltState = Claw.TILTSTATE.UP;
        this.extensionTarget = 0; // Needs to be determined experimentally
        this.pitchAngle = Pitch.PITCHPOS.DOWN;
    }


}
