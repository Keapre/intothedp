package org.firstinspires.ftc.teamcode.Utils.Files.Loggers;

import org.firstinspires.ftc.teamcode.Utils.Files.Clocks.Clock;
import org.firstinspires.ftc.teamcode.Utils.Files.FileUtils.LogWriter;

import java.util.ArrayList;
import java.util.HashMap;


public class GridLogger {

    private LogWriter writer;
    private Clock clock;
    private ArrayList<String> columnHeaders = new ArrayList<>();;
    private HashMap<String, String> rowData = new HashMap<>();
    private boolean firstLine;
    private long startTime;

    public GridLogger(LogWriter writer, Clock clock){
        this.writer = writer;
        this.clock = clock;
        startTime = clock.getCurrentTime();
        firstLine = true;
    }

    /**
     * Add a value to the logger under the specified column
     *
     * @param column
     * @param entry
     */
    public void add(String column, String entry){
        if (!columnHeaders.contains(column)) columnHeaders.add(column);
        rowData.put(column, entry);
    }

    /**
     * Add a value to the logger under the specified column
     *
     * @param column
     * @param value
     */
    public void add(String column, double value){
        add(column, String.valueOf(value));
    }

    /**
     * Write a line of data to the log.  If this is the first call to writeRow, categories are
     * written first, followed by the line of data.  Once the data is written, the logger is reset
     * and calls to add() will add values to the next line of data.
     */
    public void writeRow(){


        // Check there's data to write
        if (rowData.size() != 0){

            StringBuilder builder = new StringBuilder();

            // Initialize time
            add("Time", clock.getCurrentTime() - startTime);

            if (firstLine) {
                firstLine = false;
                for (int i=0; i < columnHeaders.size(); i++){
                    builder.append(columnHeaders.get(i));
                    if (i != columnHeaders.size() - 1) builder.append(",");
                }
                writer.writeLine(builder.toString());
            }


            builder = new StringBuilder();
            for (int i=0; i < columnHeaders.size(); i++){
                builder.append(rowData.get(columnHeaders.get(i)));
                if (i != columnHeaders.size() - 1) builder.append(",");
            }
            writer.writeLine(builder.toString());
        }

        rowData.clear();
    }

    public void stop(){
        writer.stop();
    }

    public boolean isWriting(){
        return writer.isWriting();
    }
}