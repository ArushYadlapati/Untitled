package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * This class contains all methods related to moving robots
 */
public class RobotMover {

    HardwareMap hardwareMap; //< hardware map
    Telemetry telemetry; //< telemetry logger
    AprilTagDetector aprilTagDetector; //< the april tag detector
    Vector2d lastPosition; //< the last known position of the robot

    /**
     * the constructor
     *
     * @param _hardwareMap      map of the hardware devices connected to this robot
     * @param _telemetry        telemetry logger
     * @param _aprilTagDetector the April Tag detector
     * @param _startPos         the robot start position
     */
    RobotMover(HardwareMap _hardwareMap, Telemetry _telemetry, AprilTagDetector _aprilTagDetector, Vector2d _startPos) {
        // Initialize the robot here
        // 1. Initialize the motors and set the encoder mode
        // 2. Save the telemetry, aprilTagDetector and start position

        hardwareMap = _hardwareMap;
        telemetry = _telemetry;
        aprilTagDetector = _aprilTagDetector;
        lastPosition = _startPos;

        // TODO: to be implemented later
    }

    /**
     * get the current robot orientation in the FTC field coordinate
     *
     * @return the orientation angle in the FTC field cooridnate system
     */
    public double getCurrentOrientation() {
        // Read the current robot orientation from IMU
        // TODO: to be implemented later

        return 0.0;
    }

    /**
     * get the current robot position in the FTC field coordinate
     *
     * @return the robot position
     */
    public Vector2d getCurrentPosition() {
        Vector2d pos = aprilTagDetector.getLocation();
        if (pos != null) {
            lastPosition = pos;
        }
        return lastPosition;
    }

    /**
     * turn the robot to the input orientation in the FTC field coordinate
     *
     * @param orientation the angle of the FTC field coordinate system
     * @return true (if successful) | false (otherwise)
     */
    public boolean turnTo(double orientation) {
        double angelToTurn = orientation - getCurrentOrientation();
        if (0 == angelToTurn) {
            return true;
        }

        // Rotate to the given angle

        // TODO: to be implemented later
        return false;
    }

    /**
     * drive the robot forward to the input distance
     *
     * @param distance the driving distance
     * @return true (if successful) | false (otherwise)
     */
    public boolean forwardTo(double distance) {
        Vector2d start = getCurrentPosition();
        double angle = getCurrentOrientation();

        // Drive forward to the given distance
        // TODO: to be implemented later

        // update
        Vector2d delta = new Vector2d(distance, 0).rotated(angle);
        lastPosition = lastPosition.plus(delta);

        return false;
    }

    /**
     * Move the robot along the input path
     *
     * @param path the transition points along the path
     * @return true (if successful) | false (otherwise)
     */
    public boolean moveTo(List<Vector2d> path) {
        for (Vector2d nextStop : path) {
            Vector2d currentPosition = getCurrentPosition();
            double currentOrientation = getCurrentOrientation();
            Vector2d delta = nextStop.minus(currentPosition);

            if (!turnTo(delta.angle())) {
                telemetry.addData("moveTo() Error", "Failed to turn to ", nextStop);
                return false;
            }

            if (!forwardTo(currentPosition.distTo(nextStop))) {
                telemetry.addData("moveTo() Error", "Failed to move to ", nextStop);
                return false;
            }
        }

        return true;
    }
}