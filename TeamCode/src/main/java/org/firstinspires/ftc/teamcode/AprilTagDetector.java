package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * This is a class to locate the robot position by using the April Tag detections
 */
public class AprilTagDetector {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    HardwareMap hardwareMap;
    Telemetry telemetry;

    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    //assigns variables to be used in the entire program. not assigned to functions
    private VisionPortal visionPortal;

    private VectorF finalVector;

    private final AprilTagLibrary aprilTagLibrary = getCenterStageTagLibrary();

    /**
     * The constructor
     *
     * @param _hardwareMap map of the hardware devices connected to this robot
     * @param _telemetry   telemetry logger
     */
    public AprilTagDetector(HardwareMap _hardwareMap, Telemetry _telemetry) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;

        // Initialize the April detector here
        initAprilTag();
    }

    /**
     * Get the current robot location through April tag detections
     *
     * @return the current robot position and yaw (if successful) | null (otherwise)
     */
    public Vector2d getLocation() {
        // Implement the april tag detection and distance calculation here
        List<VectorF> poses = new ArrayList<VectorF>();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        boolean tagDetected = false;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            int tagID = detection.id;
            if (detection.metadata != null) {
                tagDetected = true;

                double myTagPoseX = detection.ftcPose.x;
                double myTagPoseY = detection.ftcPose.y;
                double myTagPoseZ = detection.ftcPose.z;

                double myTagPosePitch = detection.ftcPose.pitch;
                double myTagPoseRoll = detection.ftcPose.roll;
                double myTagPoseYaw = detection.ftcPose.yaw;

                double range = detection.ftcPose.range;
                double bearing = detection.ftcPose.bearing;

                poses.add(getRobotLocation(myTagPoseYaw, myTagPoseZ, range, bearing, tagID));

            }
        }

        // If no tag is detected, return null directly
        if (!tagDetected) {
            return null;
        }

        // Average all the poses you have, getting a singular pose vector which should be more accurate
        double x = 0;
        double y = 0;
        for (int i = 0; i < poses.size(); i++) {
            x += poses.get(i).get(0);
            y += poses.get(i).get(1);
        }

        return new Vector2d(x / poses.size(), y / poses.size());
    }

    // More private functions here
    public static AprilTagLibrary getCenterStageTagLibrary() {
        return new AprilTagLibrary.Builder()
                //default Tag location
                .addTag(1, "BlueAllianceLeft", 2, new VectorF(60.25f, 41.41f, 4f), DistanceUnit.INCH, new Quaternion(0.683f, -0.183f, 0.183f, 0.683f, 0))
                .addTag(2, "BlueAllianceCenter", 2, new VectorF(60.25f, 35.41f, 4f), DistanceUnit.INCH, new Quaternion(0.683f, -0.183f, 0.183f, 0.683f, 0))
                .addTag(3, "BlueAllianceRight", 2, new VectorF(60.25f, 29.41f, 4f), DistanceUnit.INCH, new Quaternion(0.683f, -0.183f, 0.183f, 0.683f, 0))
                .addTag(4, "RedAllianceLeft", 2, new VectorF(60.25f, -29.41f, 4f), DistanceUnit.INCH, new Quaternion(0.683f, -0.183f, 0.183f, 0.683f, 0))
                .addTag(5, "RedAllianceCenter", 2, new VectorF(60.25f, -35.41f, 4f), DistanceUnit.INCH, new Quaternion(0.683f, -0.183f, 0.183f, 0.683f, 0))
                .addTag(6, "RedAllianceRight", 2, new VectorF(60.25f, -41.41f, 4f), DistanceUnit.INCH, new Quaternion(0.683f, -0.183f, 0.183f, 0.683f, 0))
                .addTag(7, "RedAudienceWallLarge", 5, new VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH, new Quaternion(0.7071f, 0, 0, -7.071f, 0))
                .addTag(8, "RedAudienceWallSmall", 2, new VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH, new Quaternion(0.7071f, 0, 0, -7.071f, 0))
                .addTag(9, "BlueAudienceWallSmall", 2, new VectorF(-70.25f, 35.125f, 4), DistanceUnit.INCH, new Quaternion(0.7071f, 0, 0, -7.071f, 0))
                .addTag(10, "BlueAudienceWallLarge", 5, new VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH, new Quaternion(0.7071f, 0, 0, -7.071f, 0))
                .build();
    }

    //Initialize the AprilTag processor.
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder().build();

        // visionPortal
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }

    private VectorF getRobotLocation(double myTagPoseYaw, double myTagPoseZ, double range, double bearing, int tagID) { //to do average vector, change "void" to vectorF
        VectorF fieldPose;
        double fieldY = range * Math.sin(Math.toRadians(bearing - myTagPoseYaw));
        double fieldX = range * Math.cos(Math.toRadians(bearing - myTagPoseYaw));
        //double finalYaw = 0-myTagPoseYaw;

        if (tagID <= 6) {
            fieldX *= -1;
            fieldY *= -1;
        }

        //TODO: Add Yaw Here
        //expression may be < 0 if i got y reversed 💀
        fieldPose = new VectorF((float) fieldX, (float) fieldY, (float) -myTagPoseZ);
        fieldPose.add(aprilTagLibrary.lookupTag(tagID).fieldPosition);
        telemetry.addData("Corresponding Field Position", fieldPose);
        return fieldPose;
    }

}