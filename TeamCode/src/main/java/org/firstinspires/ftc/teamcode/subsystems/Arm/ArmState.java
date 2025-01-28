package org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.Arm.Claw.Claw;

import java.util.function.Supplier;

@Config
public enum ArmState {
    DEFAULT(
            ()-> (double) 0,
            () -> (double) 0,
            ArmConstants.OPEN_CLAW,
            ArmConstants.ORIZONTAL_ROTATE,
            ArmConstants.MID_TILT
    ),
    HIGHBASKET(
            () -> ArmConstants.HIGHBASKET_EXTENSION,
            () -> ArmConstants.HIGHBASKET_PITCH,
            ArmConstants.CLOSE_CLAW,
            ArmConstants.ORIZONTAL_ROTATE,
            ArmConstants.UP_TILT
    ),
    INTAKING(
            () -> ArmConstants.INTAKING_EXTENSION,
            () -> ArmConstants.INTAKING_PITCH,
            ArmConstants.CLOSE_CLAW,
            ArmConstants.ORIZONTAL_ROTATE,
            ArmConstants.MID_TILT
    ),
    HighBasketTeleOp(
            () -> ArmConstants.HIGHBASKET_EXTENSION,
            () -> ArmConstants.HIGHBASKET_PITCH,
            ArmConstants.CLOSE_CLAW,
            ArmConstants.ORIZONTAL_ROTATE,
            ArmConstants.UP2_TILT
    ),
    SpecimenBar(
            () -> ArmConstants.HIGHBASKET_EXTENSION,
            () -> ArmConstants.HIGHBASKET_PITCH,
            ArmConstants.CLOSE_CLAW,
            ArmConstants.ORIZONTAL_ROTATE,
            ArmConstants.UP_TILT
    ),
    SPECIMEN(
            () -> ArmConstants.SPECIMEN_EXTENSION,
            () -> ArmConstants.SPECIMEN_PITCH,
            ArmConstants.CLOSE_CLAW,
            ArmConstants.ORIZONTAL_ROTATE,
            ArmConstants.DOWN_TILT
    ),
    START_POS(
            () -> ArmConstants.START_EXTENSION,
            () -> ArmConstants.START_PITCH,
            ArmConstants.CLOSE_CLAW,
            ArmConstants.ORIZONTAL_ROTATE,
            ArmConstants.UP_TILT
    ),
    SPECIMENGARD(
            () -> ArmConstants.SPECIMENGARD_EXTENSION,
            () -> ArmConstants.SPECIMENGARD_PITCH,
            ArmConstants.OPEN_CLAW,
            ArmConstants.ORIZONTAL_ROTATE,
            ArmConstants.GARD_TILT
    ),
    SPECIMENSLAM(
            () -> ArmConstants.SPECIMENSLAM_EXTENSION,
            () -> ArmConstants.SPECIMENSLAM_PITCH,
            ArmConstants.CLOSE_CLAW,
            ArmConstants.ORIZONTAL_ROTATE ,
            ArmConstants.UP_TILT
    );


    private final Supplier<Double> extensionSupplier;
    private final Supplier<Double> pivotSupplier;
    private Claw.CLAWPOS clawpos;
    private Claw.RotateMode rotatePos;
    private Claw.tiltMode tiltState;

    ArmState(Supplier<Double> extensionSupplier, Supplier<Double> pivotSupplier,
             Claw.CLAWPOS clawpos, Claw.RotateMode rotatePos, Claw.tiltMode tiltState) {
        this.extensionSupplier = extensionSupplier;
        this.pivotSupplier = pivotSupplier;
        this.clawpos = clawpos;
        this.rotatePos = rotatePos;
        this.tiltState = tiltState;
    }

    public double getExtensionTarget() {
        return extensionSupplier.get();
    }

    public double getPivotAngle() {
        return pivotSupplier.get();
    }

    public Claw.CLAWPOS getClawpos() {
        return clawpos;
    }

    public Claw.RotateMode getRotatePos() {
        return rotatePos;
    }

    public Claw.tiltMode getTiltState() {
        return tiltState;
    }
}
