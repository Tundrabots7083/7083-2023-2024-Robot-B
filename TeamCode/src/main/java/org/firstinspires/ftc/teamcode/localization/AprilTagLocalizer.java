package org.firstinspires.ftc.teamcode.localization;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class AprilTagLocalizer implements Localizer {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    /**
     * A map of tag IDs to their corresponding CenterStageTagData.
     */
    private Map<Integer, CenterStageTagData> tagMap;

    /**
     * The current pose estimate of the robot.
     */
    private Pose2d poseEstimate;

    private boolean foundTags;

    public AprilTagLocalizer(CameraName camera) {
        initAprilTag(camera);
        tagMap = CenterStageTagData.getTagMap();
        foundTags = false;
    }

    /**
     * Returns the most recent estimate of the robot's position on the field.
     * @return a Pose2d representing the robot's calculated position on the field
     */
    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        poseEstimate = pose2d;
    }

    /**
     * Returns null always.
     * Roadrunner docs say this is optional, so until we need it we won't implement it.
     * @return null
     */
    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    private Pose2d getPoseFromDetection(AprilTagDetection detection) {
        CenterStageTagData reference = tagMap.get(detection.id);
        if (reference != null) {
            AprilTagPoseFtc detectedPose = detection.ftcPose;

            double range = detectedPose.range;
            double bearing = detectedPose.bearing;
            double yaw = detectedPose.yaw;

            // Compute the angle from the tag to the robot, relative to the global coordinate system
            double theta = (bearing + (Math.PI / 2)) + yaw;

            // Compute the x and y coordinates of the robot in global coordinates
            double x = reference.x - (range * Math.sin(theta));
            double y = reference.y + (range * Math.cos(theta));

            // Compute the robot's heading in global coordinates
            double heading = Math.toRadians(reference.heading) + yaw;

            return new Pose2d(x, y, heading);
        }
        return null;
    }

    @Override
    public void update() {
        // Get the list of AprilTag detections
        List<AprilTagDetection> detections = aprilTag.getDetections();

        // If there are any detections, iterate through each and compute the location of the robot
        List<Pose2d> possiblePoses = new ArrayList<>();
        for (AprilTagDetection detection : detections) {
            Pose2d pose = getPoseFromDetection(detection);
            if (pose != null) {
                possiblePoses.add(pose);
            }
        }

        // Aggregate the possible poses into a single pose
        if (possiblePoses.size() > 0) {
            double x_sum = 0;
            double y_sum = 0;
            double heading_sum = 0;
            for (Pose2d possiblePose : possiblePoses) {
                x_sum += possiblePose.getX();
                y_sum += possiblePose.getY();
                heading_sum += possiblePose.getHeading();
            }
            // Compute the average of the x, y, and heading values
            double x_avg = x_sum / possiblePoses.size();
            double y_avg = y_sum / possiblePoses.size();
            double heading_avg = heading_sum / possiblePoses.size();

            // Set the pose estimate to the average of the possible poses
            Pose2d poseEstimate = new Pose2d(x_avg, y_avg, heading_avg);
            setPoseEstimate(poseEstimate);
            // Set foundTags to true
            foundTags = true;
        }
        else {
            // If there are no detections, set foundTags to false
            foundTags = false;
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag(CameraName camera) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera
        builder.setCamera(camera);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Enable the april tag processor
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    public boolean foundTags() {
        return foundTags;
    }
}
