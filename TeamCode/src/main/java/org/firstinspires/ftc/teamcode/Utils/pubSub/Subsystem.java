package org.firstinspires.ftc.teamcode.Utils.pubSub;

public interface Subsystem {
    /**
     * Run control code (e.g., read sensors and update motors)
     * TODO: Return telemetry.
     */
    void update();

    default void stop() {
    }
}