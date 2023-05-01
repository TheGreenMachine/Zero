package com.team1816.lib.util.logUtil;

import com.team1816.season.configuration.Constants;

import java.io.File;
import java.time.Instant;
import java.util.*;

public class RioLogManager {

    /**
     * Deletes old WPIlogs.
     * Prioritizes deletion of the oldest, non-match logs.
     */
    public static void sweepLogs(String logFileDir) {
        File root = new File("/");
        File logDir = new File(logFileDir);

        int errors = 0;

        List<File> nonMatchLogs = Arrays.asList(
            Objects.requireNonNull(logDir.listFiles((dir1, name) ->
                    !name.contains("Q") &&
                    !name.contains("E") &&
                    !name.contains("P")
            )));
        nonMatchLogs.sort(Comparator.comparingLong(File::lastModified)); // Smaller value indicates older file
        nonMatchLogs.forEach(file -> {
            if(Instant.now().toEpochMilli() - file.lastModified() < (long)(8.64 * (10^7))) { // If file was created within a day
                nonMatchLogs.remove(file);
            }
        });

        List<File> matchLogs = Arrays.asList(
            Objects.requireNonNull(logDir.listFiles((dir1, name) ->
                name.contains("Q") ||
                    name.contains("E") ||
                    name.contains("P")
            )));
        matchLogs.sort(Comparator.comparingLong(File::lastModified)); // Smaller value indicates older file
        matchLogs.forEach(file -> {
            if(Instant.now().toEpochMilli() - file.lastModified() < (long)(8.64 * (10^7))) { // If file was created within a day
                nonMatchLogs.remove(file);
            }
        });



        if (root.getUsableSpace() < Constants.kUsableDiskSpace) {
            GreenLogger.log("!!! Disk usage at " + (root.getUsableSpace() / root.getTotalSpace())*100 + "% !!!");
        } else {
            GreenLogger.log("Disk usage at " + (root.getUsableSpace() / root.getTotalSpace())*100 + "%");
        }

        while (root.getUsableSpace() < Constants.kUsableDiskSpace) {
            if(errors >= 3){
                break;
            }
            if (!nonMatchLogs.isEmpty()) {
                File oldestLog = nonMatchLogs.get(0);

                if (oldestLog.delete()) {
                    nonMatchLogs.remove(0);
                    GreenLogger.log("Deleting file: " + oldestLog.getName());

                    if(nonMatchLogs.isEmpty() && root.getUsableSpace() < Constants.kUsableDiskSpace){
                        GreenLogger.log("No more non-match logs available for deletion, moving on to match logs");
                    }
                } else {
                    GreenLogger.log("Error deleting file" + oldestLog.getName());
                    errors++;
                }
            } else {
                File oldestLog = matchLogs.get(0);

                if(oldestLog.delete()) {
                    matchLogs.remove(0);
                    GreenLogger.log("Deleting file: " + oldestLog.getName());

                    if(matchLogs.isEmpty() && root.getUsableSpace() < Constants.kUsableDiskSpace){
                        GreenLogger.log("Disk usage still >= 70% with no more files created > 1 day ago, please manually choose and delete files.");
                        break;
                    }
                } else {
                    GreenLogger.log("Error deleting file" + oldestLog.getName());
                    errors++;
                }
            }
        }
        if(errors >= 3) {
            GreenLogger.log("3 or more errors deleting files encountered, please manually investigate.");
        } else if(root.getUsableSpace() < Constants.kUsableDiskSpace) {
            GreenLogger.log("Disk usage at " + (root.getUsableSpace() / root.getTotalSpace())*100 + "%");
        }
    }
}

