package com.team1816.lib.util.visionUtil;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * A lightweight fiducial marker identification utility
 */

public class VisionPoint {

    public int id; // -2 if not detected
    public Transform3d cameraToTarget;
    public double weight;

    public VisionPoint() {
        id = 0;
        cameraToTarget = new Transform3d();
        weight = 0;
    }

    public VisionPoint(int i, Transform3d targetTransform) {
        id = i;
        cameraToTarget = targetTransform;
        weight = 0;
    }

    public double getX() {
        return cameraToTarget.getX();
    }

    public double getY() {
        return cameraToTarget.getY();
    }

    public double getZ() {
        return cameraToTarget.getZ();
    }

    public String toString() {
        return "id: " + id + " camera to target: " + cameraToTarget;
    }
}
