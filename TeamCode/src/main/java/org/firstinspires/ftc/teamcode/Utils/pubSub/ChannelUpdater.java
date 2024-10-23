package org.firstinspires.ftc.teamcode.Utils.pubSub;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.Dashboard.TelemetryUtil;
import org.firstinspires.ftc.teamcode.Utils.pubSub.Channel;
import org.firstinspires.ftc.teamcode.Utils.pubSub.Subsystem;
import org.firstinspires.ftc.teamcode.Utils.pubSub.Update;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class ChannelUpdater implements Channel {

    public boolean ENABLED = true;

    FileWriter writer = null;
    public ChannelUpdater() {
        File file = new File("/sdcard/FIRST/log.txt");

        // Check if the file exists and delete it
        if (file.exists()) {
            boolean deleted = file.delete();
            if (deleted) {

                TelemetryUtil.packet.put("File Status", "File deleted successfully");
            } else {
                TelemetryUtil.packet.put("File Status", "Failed to delete file");
            }
        } else {
            TelemetryUtil.packet.put("File Status", "File does not exist");
        }

        try {
            // Create the file
            if (file.createNewFile()) {
                TelemetryUtil.packet.put("File Status", "File created successfully");
            } else {
                TelemetryUtil.packet.put("File Status", "File already exists");
            }

            // Write some initial content to the file (optional)
            writer = new FileWriter(file, true);
            writer.write("Log Start\n");
            writer.close();

        } catch (IOException e) {
            TelemetryUtil.packet.put("File Status", "An error occurred: " + e.getMessage());
        }
    }
    @Override
    public void receive(String TAG, Update update, String description){

        if(ENABLED) {
            try {
                writer.write( System.currentTimeMillis() / 1000 + ": " +  TAG + " " +  description);
                writer.close();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }

            for(Subsystem sub : update.getSubsystems()) {
                sub.UPDATABLE = true;
            }

            update.getAction().run();
        }
    }

    @Override
    public void stop() {
        ENABLED = false;
    }

    @Override
    public void enable() {
        ENABLED = true;
    }

    @Override
    public void update() {

    }
}
