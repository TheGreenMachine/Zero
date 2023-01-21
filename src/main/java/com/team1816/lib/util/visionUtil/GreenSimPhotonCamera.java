/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.team1816.lib.util.visionUtil;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonVersion;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@SuppressWarnings("unused")
public class GreenSimPhotonCamera extends GreenPhotonCamera {

    public List<NetworkTableEntry> targetList = new ArrayList<NetworkTableEntry>();

    public NetworkTableEntry targetListEntry1; //pain
    public NetworkTableEntry targetListEntry2;
    public NetworkTableEntry targetListEntry3;
    public NetworkTableEntry targetListEntry4;
    public NetworkTableEntry targetListEntry5;
    public NetworkTableEntry targetListEntry6;
    public NetworkTableEntry targetListEntry7;
    public NetworkTableEntry targetListEntry8;
    public NetworkTableEntry targetListEntry9;

    private final NetworkTableEntry versionEntry;
    private final NetworkTableEntry latencyMillisEntry;
    private final NetworkTableEntry hasTargetEntry;

    /**
     * Constructs a Simulated PhotonCamera from a root table.
     *
     * @param instance   The NetworkTableInstance to pull data from. This can be a custom instance in
     *                   simulation, but should *usually* be the default NTInstance from
     *                   NetworkTableInstance::getDefault
     * @param cameraName The name of the camera, as seen in the UI.
     */
    public GreenSimPhotonCamera(NetworkTableInstance instance, String cameraName) {
        super(instance, cameraName);
        latencyMillisEntry = rootTable.getEntry("latencyMillis");
        hasTargetEntry = rootTable.getEntry("hasTargetEntry");

        targetListEntry1 = rootTable.getEntry("targetListEntry1");
        targetListEntry2 = rootTable.getEntry("targetListEntry2");
        targetListEntry3 = rootTable.getEntry("targetListEntry3");
        targetListEntry4 = rootTable.getEntry("targetListEntry4");
        targetListEntry5 = rootTable.getEntry("targetListEntry5");
        targetListEntry6 = rootTable.getEntry("targetListEntry6");
        targetListEntry7 = rootTable.getEntry("targetListEntry7");
        targetListEntry8 = rootTable.getEntry("targetListEntry8");
        targetListEntry9 = rootTable.getEntry("targetListEntry9");

        targetList.add(targetListEntry1);
        targetList.add(targetListEntry2);
        targetList.add(targetListEntry3);
        targetList.add(targetListEntry4);
        targetList.add(targetListEntry5);
        targetList.add(targetListEntry6);
        targetList.add(targetListEntry7);
        targetList.add(targetListEntry8);
        targetList.add(targetListEntry9);

        versionEntry = rootTable.getEntry("versionEntry");
        // Sets the version string so that it will always match the current version
        versionEntry.setString(PhotonVersion.versionString);
    }

    /**
     * Constructs a Simulated PhotonCamera from the name of the camera.
     *
     * @param cameraName The nickname of the camera (found in the PhotonVision UI).
     */
    public GreenSimPhotonCamera(String cameraName) {
        this(NetworkTableInstance.getDefault(), cameraName);
    }

    /**
     * Simulate one processed frame of vision data, putting one result to NT.
     *
     * @param latencyMillis Latency of the provided frame
     * @param targets       Each target detected
     */
    public void submitProcessedFrame(
        double latencyMillis,
        PhotonTrackedTarget... targets
    ) {
        submitProcessedFrame(latencyMillis, Arrays.asList(targets));
    }

    /**
     * Simulate one processed frame of vision data, putting one result to NT.
     *
     * @param latencyMillis Latency of the provided frame
     * @param sortMode      Order in which to sort targets
     * @param targets       Each target detected
     */
    public void submitProcessedFrame(
        double latencyMillis,
        PhotonTargetSortMode sortMode,
        PhotonTrackedTarget... targets
    ) {
        submitProcessedFrame(latencyMillis, sortMode, Arrays.asList(targets));
    }

    /**
     * Simulate one processed frame of vision data, putting one result to NT.
     *
     * @param latencyMillis Latency of the provided frame
     * @param targetList    List of targets detected
     */
    public void submitProcessedFrame(
        double latencyMillis,
        List<PhotonTrackedTarget> targetList
    ) {
        submitProcessedFrame(latencyMillis, null, targetList);
    }

    /**
     * Simulate one processed frame of vision data, putting one result to NT.
     *
     * @param latencyMillis       Latency of the provided frame
     * @param sortMode            Order in which to sort targets
     * @param submittedTargetList List of targets detected
     */
    public void submitProcessedFrame(
        double latencyMillis,
        PhotonTargetSortMode sortMode,
        List<PhotonTrackedTarget> submittedTargetList
    ) {
        latencyMillisEntry.setDouble(latencyMillis);

        if (sortMode != null) {
            submittedTargetList.sort(sortMode.getComparator());
        }

        PhotonPipelineResult newResult = new PhotonPipelineResult(
            latencyMillis,
            submittedTargetList
        );
        var newPacket = new Packet(newResult.getPacketSize());
        newResult.populatePacket(newPacket);
        rawBytesEntry.setRaw(newPacket.getData());

        boolean hasTargets = newResult.hasTargets();
        hasTargetEntry.setBoolean(hasTargets);
        if (!hasTargets) {
            for (int i = 0; i < targetList.size(); i++) {
                targetList.get(i).setString("None");
            }
        } else {
            var targets = newResult.getTargets();

            for (int i = 0; i < 9; i++) {
                if (i >= targets.size() - 1) {
                    targetList.get(i).setString("None");
                } else {
                    targetList
                        .get(i)
                        .setString(PhotonTrackTargetToString(targets.get(i)));
                }
            }
        }
    }

    private static String PhotonTrackTargetToString(PhotonTrackedTarget t) {
        return (
            "Id: " +
                t.getFiducialId() +
                //            "; yaw= " +
                //            (int) t.getYaw() + // If yaw seems weird that's because it's an int
                "; X: " +
                roundAvoid(t.getBestCameraToTarget().getX(), 3) +
                "; Y: " +
                roundAvoid(t.getBestCameraToTarget().getY(), 3) +
                "; Z: " +
                roundAvoid(t.getBestCameraToTarget().getZ(), 3)
        );
    }

    public static double roundAvoid(double value, int places) {
        double scale = Math.pow(10, places);
        return Math.round(value * scale) / scale;
    }
}
