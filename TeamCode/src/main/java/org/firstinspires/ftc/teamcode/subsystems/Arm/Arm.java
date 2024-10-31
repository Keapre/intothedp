package org.firstinspires.ftc.teamcode.subsystems.Arm;

import android.util.JsonReader;

import com.google.gson.Gson;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.FileNotFoundException;
import java.io.FileReader;

public class Arm {

    public boolean clawChanged = true,armChanged = true,pitchChanged = true;
    public Claw claw;
    public Extension extension;
    public Pitch pitch;
    public Arm(HardwareMap hardwareMap,boolean isAuto) throws FileNotFoundException {
        claw = new Claw(hardwareMap,isAuto);
        extension = new Extension(hardwareMap,isAuto);
        pitch = new Pitch(hardwareMap,isAuto);

    }

    public void setExtendedIntake() {
        extension.target = 500;
        claw.clawPos = Claw.CLAWPOS.OPEN;
        claw.rotateState = Claw.ROTATESTATE.DEFAULT;
        claw.tiltState = Claw.TILTSTATE.DOWN;

        pitch.target = Pitch.EXTENSIONPOS.DOWN;

    }

    public void update() {
        claw.update();
        extension.update();
        pitch.update();
    }
}
