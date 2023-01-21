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

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;
import java.util.List;

public class GreenSimVisionSystem {

    GreenSimPhotonCamera cam;

    double camHorizFOVDegrees;
    double camVertFOVDegrees;
    double cameraHeightOffGroundMeters;
    double maxLEDRangeMeters;
    double camPitchDegrees;
    int cameraResWidth;
    int cameraResHeight;
    double minTargetArea;
    Transform2d cameraToRobot;

    ArrayList<GreenSimVisionTarget> targetList;

    /**
     * Create a simulated vision system involving a camera and coprocessor mounted on a mobile robot
     * running PhotonVision, detecting one or more targets scattered around the field. This assumes a
     * fairly simple and distortion-less pinhole camera model.
     *
     * @param camName                     Name of the PhotonVision camera to create. Align it with the settings you use in
     *                                    the PhotonVision GUI.
     * @param camVertFOVDegrees           Vertical Field of View of the camera used. Align it with the
     *                                    manufacturer specifications, and/or whatever is configured in the PhotonVision Setting
     *                                    page.
     * @param camHorizFOVDegrees          Horizontal Field of View of the camera used. Align it with the
     *                                    manufacturer specifications, and/or whatever is configured in the PhotonVision Setting
     *                                    page.
     * @param camPitchDegrees             pitch of the camera's view axis back from horizontal. Make this the same
     *                                    as whatever is configured in the PhotonVision Setting page.
     * @param cameraToRobot               Pose Transform to move from the camera's mount position to the robot's
     *                                    position
     * @param cameraHeightOffGroundMeters Height of the camera off the ground in meters
     * @param maxLEDRangeMeters           Maximum distance at which your camera can illuminate the target and
     *                                    make it visible. Set to 9000 or more if your vision system does not rely on LED's.
     * @param cameraResWidth              Width of your camera's image sensor in pixels
     * @param cameraResHeight             Height of your camera's image sensor in pixels
     * @param minTargetArea               Minimum area that the target should be before it's recognized as a
     *                                    target by the camera. Match this with your contour filtering settings in the PhotonVision
     *                                    GUI.
     */
    public GreenSimVisionSystem( // not using diag FOV to calculate horiz/vert FOV b/c calculated numbers don't match official camera properties
                                 String camName,
                                 double camHorizFOVDegrees,
                                 double camVertFOVDegrees,
                                 double camPitchDegrees,
                                 Transform2d cameraToRobot,
                                 double cameraHeightOffGroundMeters,
                                 double maxLEDRangeMeters,
                                 int cameraResWidth,
                                 int cameraResHeight,
                                 double minTargetArea
    ) {
        this.camPitchDegrees = camPitchDegrees;
        this.cameraToRobot = cameraToRobot;
        this.cameraHeightOffGroundMeters = cameraHeightOffGroundMeters;
        this.maxLEDRangeMeters = maxLEDRangeMeters;
        this.cameraResWidth = cameraResWidth;
        this.cameraResHeight = cameraResHeight;
        this.minTargetArea = minTargetArea;

        // Calculate horizontal/vertical FOV by similar triangles
        double hypotPixels = Math.hypot(cameraResWidth, cameraResHeight);
        this.camHorizFOVDegrees = camHorizFOVDegrees;
        this.camVertFOVDegrees = camVertFOVDegrees;

        cam = new GreenSimPhotonCamera(camName);
        targetList = new ArrayList<>();
    }

    /**
     * Add a target on the field which your vision system is designed to detect. The PhotonCamera from
     * this system will report the location of the robot relative to the subset of these targets which
     * are visible from the given robot position.
     *
     * @param target Target to add to the simulated field
     */
    public void addSimVisionTarget(GreenSimVisionTarget target) {
        targetList.add(target);
    }

    /**
     * Adjust the camera position relative to the robot. Use this if your camera is on a gimbal or
     * turret or some other mobile platform.
     *
     * @param newCameraToRobot   New Transform from the robot to the camera
     * @param newCamHeightMeters New height of the camera off the floor
     * @param newCamPitchDegrees New pitch of the camera axis back from horizontal
     */
    public void moveCamera(
        Transform2d newCameraToRobot,
        double newCamHeightMeters,
        double newCamPitchDegrees
    ) {
        this.cameraToRobot = newCameraToRobot;
        this.cameraHeightOffGroundMeters = newCamHeightMeters;
        this.camPitchDegrees = newCamPitchDegrees;
    }

    /**
     * Periodic update. Call this once per frame of image data you wish to process and send to
     * NetworkTables
     *
     * @param robotPoseMeters current pose of the robot on the field. Will be used to calculate which
     *                        targets are actually in view, where they are at relative to the robot, and relevant
     *                        PhotonVision parameters.
     */
    public void processFrame(Pose2d robotPoseMeters) {
        Pose2d cameraPos = robotPoseMeters.transformBy(cameraToRobot.inverse());
        ArrayList<PhotonTrackedTarget> visibleTgtList = new ArrayList<>(
            targetList.size()
        );

        targetList.forEach(
            tgt -> {
                var camToTargetTransform = new Transform3d(
                    new Pose3d(
                        cameraPos.getX(),
                        cameraPos.getY(),
                        cameraHeightOffGroundMeters,
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(camPitchDegrees),
                            cameraPos.getRotation().getRadians()
                        )
                    ),
                    new Pose3d(
                        tgt.targetPos.getX(),
                        tgt.targetPos.getY(),
                        tgt.targetHeightMeters,
                        new Rotation3d()
                    )
                );
                double distAlongGroundMeters = camToTargetTransform
                    .getTranslation()
                    .getNorm();
                double distVerticalMeters =
                    tgt.targetHeightAboveGroundMeters - this.cameraHeightOffGroundMeters;
                double distMeters = Math.hypot(distAlongGroundMeters, distVerticalMeters);

                double area = tgt.tgtAreaMeters2 / getM2PerPx(distAlongGroundMeters);

                // 2D yaw mode considers the target as a point, and should ignore target rotation.
                // Photon reports it in the correct robot reference frame.
                // IE: targets to the left of the image should report negative yaw.
                double yawDegrees =
                    -1.0 *
                        Units.radiansToDegrees(
                            Math.atan2(
                                camToTargetTransform.getTranslation().getY(),
                                camToTargetTransform.getTranslation().getX()
                            )
                        );
                double pitchDegrees =
                    Units.radiansToDegrees(
                        Math.atan2(distVerticalMeters, distAlongGroundMeters)
                    ) -
                        this.camPitchDegrees;

                if (camCanSeeTarget(distMeters, yawDegrees, pitchDegrees, area)) {
                    visibleTgtList.add(
                        new PhotonTrackedTarget(
                            yawDegrees,
                            pitchDegrees,
                            area,
                            0.0,
                            tgt.fiducialID,
                            new Transform3d(
                                new Translation3d(
                                    camToTargetTransform.getX(),
                                    camToTargetTransform.getY(),
                                    distVerticalMeters
                                ),
                                new Rotation3d()
                            ),
                            new Transform3d(
                                new Translation3d(
                                    camToTargetTransform.getX(),
                                    camToTargetTransform.getY(),
                                    distVerticalMeters
                                ),
                                new Rotation3d()
                            ),
                            0.25,
                            List.of( // not utilizing target corners
                                new TargetCorner(0, 0),
                                new TargetCorner(0, 0),
                                new TargetCorner(0, 0),
                                new TargetCorner(0, 0)
                            ),
                            List.of(
                                new TargetCorner(0, 0),
                                new TargetCorner(0, 0),
                                new TargetCorner(0, 0),
                                new TargetCorner(0, 0)
                            )
                        )
                    );
                }
            }
        );

        cam.submitProcessedFrame(0.0, PhotonTargetSortMode.Largest, visibleTgtList);
    }

    double getM2PerPx(double dist) {
        double widthMPerPx =
            2 *
                dist *
                Math.tan(Units.degreesToRadians(this.camHorizFOVDegrees) / 2) /
                cameraResWidth;
        double heightMPerPx =
            2 *
                dist *
                Math.tan(Units.degreesToRadians(this.camVertFOVDegrees) / 2) /
                cameraResHeight;
        return widthMPerPx * heightMPerPx;
    }

    boolean camCanSeeTarget(double distMeters, double yaw, double pitch, double area) {
        boolean inRange = (distMeters < this.maxLEDRangeMeters);
        boolean inHorizAngle = Math.abs(yaw) < (this.camHorizFOVDegrees / 2);
        boolean inVertAngle = Math.abs(pitch) < (this.camVertFOVDegrees / 2);
        boolean targetBigEnough = area > this.minTargetArea;
        return (inRange && inHorizAngle && inVertAngle && targetBigEnough);
    }
}
