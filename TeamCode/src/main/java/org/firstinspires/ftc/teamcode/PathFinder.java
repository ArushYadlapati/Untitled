package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.List;

/**
 * a utility class to find the shortest path to move the robot to a destination
 * point without hitting the trusses.
 */
public class PathFinder {

    /**
     * the default constructor
     */
    public PathFinder() {
        // Initialize the path finder class, including the array of safe paths here

        // TODO: to be implemented later
    }

    /**
     * Find the shortest path to safely move from the source to destination
     * @param source the source coordinate
     * @param destination the destination coordinates
     * @return the list of transition points to the destination (if found) | null (otherwise)
     */
    public List<Vector2d> findShortestPath(Vector2d source, Vector2d destination) {
        // Implement the path searching function here

        // TODO: to be implemented later
        return null;
    }

    /**
     * Check whether the two input points are on the same side of the field
     * @param point1 the coordinate of the first point
     * @param point2 the coordinate of the 2nd point
     *
     * @return true (if the two points are on the same side) | false (otherwise)
     */
    private boolean isOnSameSide(Vector2d point1, Vector2d point2) {
        return point1.getX() * point2.getX() >= 0;
    }

    // More private functions here
}