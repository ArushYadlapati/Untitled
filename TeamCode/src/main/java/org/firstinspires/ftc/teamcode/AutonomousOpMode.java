package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "AutonomousOpMode", group = "Autonomous") // Without this, this file will not show in the Autonomous section of the REV Driver Hub.
@Config
// Note that REV Driver Hub and REV Driver Station are synonymous.
public class AutonomousOpMode extends LinearOpMode {

    // Defines 4 Mecanum Wheel Motors, and then imu, which is set to IMU.
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declares motors using ID's that match the configuration on the REV Control Hub.
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft"); // Front Left Motor.
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft"); // Back Left Motor.
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight"); // Front Right Motor.
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight"); // Back Right Motor.

        // Set the zero power behavior to BRAKE for all motors.
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Sets all motors to use encoders.
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reset all motor encoder tick counts to 0.
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reverse the right side motors since we are using mecanum wheels.
        // Reverse left motors if you are using NeveRests.
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Declares IMU for the BHI260 IMU, which is on newer Control Hubs.
         * The older BNO055 IMU should not be used on newer Control Hubs.
         *
         * IMPORTANT:
         * For this code, the Control Hub needs to be mounted on one of the three orthogonal (right angle) planes
         * (X/Y, X/Z or Y/Z) and that the Hub has only been rotated in a range of 90 degree increments.
         * If the Control Hub is mounted on a surface angled at some non-90 Degree multiple (like 45 degrees) look at
         * the alternative SensorImuNonOrthogonal sample in:
         * FtcRobotController > src > main > java > org > firstinspires > ftc > robotcontroller > external > samples > SensorSensorImuNonOrthogonal
         *
         * This "Orthogonal" requirement means that:
         *
         * 1) The Logo printed on the top of the Hub can ONLY be pointing in one of six directions:
         *    FORWARD, BACKWARD, UP, DOWN, LEFT and RIGHT.
         *
         * 2) The USB ports can only be pointing in one of the same six directions:<br>
         *    FORWARD, BACKWARD, UP, DOWN, LEFT and RIGHT.
         */

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(

                /*
                 * Assumes Control Hub orientation is in the Default Orientation, which
                 * is when the Control Hub is mounted horizontally with the printed logo
                 * pointing UP and the USB port pointing FORWARD.
                 */

                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot)); // Initializes IMU with the given robot orientation (UP, FORWARD).

        // Wait for the start button to be pressed on the Driver Station.
        waitForStart();

        imu.resetYaw();

        // Autonomous Code goes here:
        //turn(90, 0.25);
        encoderMove(0.0,12.0,0.25);
        encoderMove(180.0,12.0,0.25);
        encoderMove(90.0,12.0,0.25);
        encoderMove(-90.0,12.0,0.25);
        telemetry.update();
    }

    // Moves the robot in a specified direction/angle for a given distance in inches.
    // Works best at ~15-40% maximum speed.
    private void encoderMove(double angle, double distanceInInches, double speed) {
        // Reset all motor encoder tick counts to 0.
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Move the robot in the specified direction.
        double forward = Math.cos(Math.toRadians(angle));
        double sideways = Math.sin(Math.toRadians(angle));

        double rotation = 0; // There is no rotation since it is a simple strafe.

        // Calculate motor powers using mecanum drive kinematics.
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = sideways * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = sideways * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotation), 1);
        double frontLeftPower = ((rotY + rotX + rotation) / denominator) * speed;
        double backLeftPower = ((rotY - rotX + rotation) / denominator) * speed;
        double frontRightPower = ((rotY - rotX - rotation) / denominator) * speed;
        double backRightPower = ((rotY + rotX - rotation) / denominator) * speed;
        double encoderTicks = ((1.0 / ((Math.pow((Math.PI * (96.0/2.0)),2) / 25.4) / ((((1+(46.0/17.0))) * (1+(46.0/11.0))) * 28))) / 12) * distanceInInches;
        // int encoderTicks = 537 // (Fail Safe without calculations as int)

        // Calculate the average target encoder position for all motors.
        int averageMotorPosition = ((motorFrontLeft.getCurrentPosition() + motorBackLeft.getCurrentPosition() + motorFrontRight.getCurrentPosition() + motorBackRight.getCurrentPosition()) / 4) * -1;

        while (opModeIsActive() && !((averageMotorPosition <= (encoderTicks + 10)) && (averageMotorPosition >= (encoderTicks - 10)))) {
            // Calculate new averageMotorPosition
            averageMotorPosition = (motorFrontLeft.getCurrentPosition() + motorBackLeft.getCurrentPosition() + motorFrontRight.getCurrentPosition() + motorBackRight.getCurrentPosition()) / 4;
            // Set motor powers.
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
        }
        stopRobot();
        telemetry.addData("motorFrontLeft Position", motorFrontLeft.getCurrentPosition());
        telemetry.addData("motorBackLeft Position", motorBackLeft.getCurrentPosition());
        telemetry.addData("motorFrontRight Position", motorFrontRight.getCurrentPosition());
        telemetry.addData("motorBackRight Position", motorBackRight.getCurrentPosition());
        telemetry.update();
    }

    public void turn(double targetYaw, double speed) {
        /*
         * The yaw value of the robot INCREASES as the robot is rotated Counter Clockwise.
         * The Control Hub's yaw value ranges from 0 to 180 degrees counterclockwise, and
         * 180 to 0 degrees clockwise.
         *
         * The speed and targetYaw are negated here because that effectively reverses the Control Hub's
         * yaw range, making it now range from 0 to 180 degrees counterclockwise, and
         * 180 to 0 degrees clockwise.
         */

        double targetYawDegrees = -targetYaw;
        double power = -speed;

        // Define the allowable error for stopping the turn.
        double allowableError = 1.0;

        // Calculate the initial difference between the target yaw and the current yaw.
        double yawDifference = targetYawDegrees - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Determine the direction of the turn based on the sign of the power.
        double direction = Math.signum(power);

        // Continue turning until the robot reaches the target yaw within the allowable error.
        while (opModeIsActive() && Math.abs(yawDifference) > allowableError) {
            // Recalculate the yaw difference in each iteration.
            yawDifference = targetYawDegrees - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Calculate the turn power based on the yaw difference and the specified speed.
            double turnPower = Math.min(Math.abs(yawDifference / 10), Math.abs(power)) * direction;

            // Set motor powers to achieve the turn.
            motorFrontLeft.setPower(turnPower);
            motorBackLeft.setPower(turnPower);
            motorFrontRight.setPower(-turnPower);
            motorBackRight.setPower(-turnPower);

            /*
             * Adds Telemetry data for monitoring.
             * Target yaw angle,
             * Yaw (Z) velocity (in Def/Sec),
             * Current yaw angle.
             */

            telemetry.addData("Target Yaw", targetYawDegrees);
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
            telemetry.addData("Current Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        // Stop the robot once the turn is complete.
        stopRobot();
    }

    // Stops the robot by setting all motor powers to 0, thus stopping the robot with ZeroPowerBehavior.BRAKE).
    private void stopRobot() {
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }
}