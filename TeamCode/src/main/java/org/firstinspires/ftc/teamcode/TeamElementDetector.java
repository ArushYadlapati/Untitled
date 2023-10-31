package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class provides the APIs to detect the location of the team elements
 */
public class TeamElementDetector {

    HardwareMap hardwareMap; ///< hardware map
    Telemetry telemetry; ///< telemetry logger
    Vector2d startPos; ///< the robot start position

    /**
     * the full constructor
     *
     * @param _hardwareMap the hardware map
     * @param _telemetry the telemetry logger
     * @param _startPos the starting
     */
    public TeamElementDetector(HardwareMap _hardwareMap, Telemetry _telemetry, Vector2d _startPos) {
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;
        startPos = _startPos;

        // TODO: to add more initialization codes here
    }

    /**
     * Get the position of the team element
     *
     * @return the field coordinate of the team element (if found) | null (otherwise)
     */
    public Vector2d getTeamElementPosition() {
        // Find which stripe the team element is placed on
        // convert the stripe index to the stripe coordinate based on the start position

        // TODO: to be implemented later

        return null;
    }
}