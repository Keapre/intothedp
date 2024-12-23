package org.firstinspires.ftc.teamcode.Utils.Files.FileUtils;

public interface LogWriter {
    void writeLine(String line);
    void stop();
    boolean isWriting();
}
