package com.team1816.lib.util.logUtil;

import com.team1816.season.configuration.Constants;
import edu.wpi.first.wpilibj.DriverStation;

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
        if(root.getUsableSpace() < (long) (Constants.kLoggingDiskPartitionRatio * root.getTotalSpace())){
            GreenLogger.log("!!! Disk usage at " + ((double) root.getUsableSpace()/root.getTotalSpace()) * 100 + "% !!!");
        }
        while (root.getUsableSpace() < (long) (Constants.kLoggingDiskPartitionRatio * root.getTotalSpace())) {
            GreenLogger.log("Disk usage at " + ((double) root.getUsableSpace()/root.getTotalSpace()) * 100 + "%");
            File oldestLog = null, logDir = new File(logFileDir);
            long ols = Long.MAX_VALUE;
            for (String f: Objects.requireNonNull(logDir.list())) {
                File cur = new File(f);
                boolean olderThanDay = Instant.now().toEpochMilli() - cur.lastModified() > (long)(8.64 * (10^7));
                // Keeps official run logs (practice, qualification, elimination) and files created within 24 hours
                if (!(f.contains("P") || f.contains("Q") || f.contains("E")) && ols > cur.lastModified() && olderThanDay) { // smaller value indicates older file
                    ols = cur.lastModified();
                    oldestLog = cur;
                }
            }
            if (oldestLog != null && oldestLog.delete()) {
                GreenLogger.log("Deleting File: " + oldestLog);
            } else {
                GreenLogger.log("Unable to Delete Log Files - Manual Deletion Required");
                DriverStation.reportError("Allotted Disk Space Exceeded - Unable to Delete Log Files", true);
                break;
            }
        }
    }
}

