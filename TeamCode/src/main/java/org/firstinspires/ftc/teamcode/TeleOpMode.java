// Please note that this code uses the Logitech F310 as its controller.
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp // Without this, this file will not show in the TeleOp section of the REV Driver Hub.
// Note that REV Driver Hub and REV Driver Station are synonymous.
public class TeleOpMode extends LinearOpMode {

    // Defines 4 Mecanum Wheel Motors, and then the  Motor.
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor inScoWheel;
    DcMotorEx armMotor;

    // Creates IMU that is set to imu.
    IMU imu;

    String last_button = ""; // Variable that stores the last gamepad1 press/call, which is displayed on REV control hub using telemetry.addData();
    boolean isFieldCentric = true; // Sets the default robot movement mode to field-centric mode when TeleOp is initialized on the REV Driver Hub.
    boolean backFlapFlag = false; // Sets the default back flap position to closed at start.
    double launch_position = 0.0; // Launch position is a servo position where the servo droneLauncher will release the rubber band, launching the drone.
    double hold_position = 0.6; // Hold position is a servo position where the servo droneLauncher will hold the rubber band in place.
    double accelerationFactor = 0.1; // Sets the default movement speed to 15% (0.15).
    double wheel_speed = 0.5; // Variable that controls what speed the inSco (intake + scoring) wheel moves at.
    int target_arm_position = 0; // Variable that controls what encoder position the arm should move to at the start of code.
    double arm_speed = 0.5; // Variable that controls the speed of the arm.

    @Override
    public void runOpMode() throws InterruptedException {
        // Declares motors using ID's that match the configuration on the REV Control Hub.
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft"); // Front Left Motor.
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft"); // Back Left Motor.
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight"); // Front Right Motor.
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight"); // Back Right Motor.
        inScoWheel = hardwareMap.dcMotor.get("inScoWheel"); // InSco Motor.
        armMotor = hardwareMap.get(DcMotorEx.class, "Front_Cage"); // Arm Motor.

        // Sets all motors to use encoders.
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the zero power behavior to BRAKE for all motors.
        motorFrontLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        inScoWheel.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // Sets all motors to use encoders.
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Reverse the right side motors since we are using mecanum wheels.
        // Reverse left motors if you are using NeveRests.
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Defines & declares servos.
        Servo droneLauncher = hardwareMap.servo.get("testDroneLauncher"); // Servo to launch drone.
        Servo backFlap = hardwareMap.servo.get("backFlap"); // Servo to control the back flap, which allows the robot to score from the back.

        // Sets both directions of the servo to forward.
        backFlap.setDirection(Servo.Direction.FORWARD);
        droneLauncher.setDirection(Servo.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu"); // Retrieves the IMU from the hardware map.

        // Adjusts the orientation parameters to match the robot (note that IMU is set to imu).
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters); // Without this, the REV Hub's orientation is assumed to be logo up / USB forward.


        // Adds telemetry to the Driver Station.
        telemetry.addData("Status", "Initialized"); // Adds Initialized Status.
        telemetry.addData("Mode", "Field-Centric"); // Since the default mode is Field-Centric, sets Field-Centric to be the mode that is added to REV Driver Hub.
        telemetry.update();

        waitForStart(); // Wait for the game to start (driver presses PLAY).

        imu.resetYaw(); // Resets imu at the start of code.

        droneLauncher.setPosition(hold_position); // Moves servo droneLauncher to hold position if not already in hold position at the start of the code.

        // backFlap.setPosition(0);// Moves servo backFlap to closed position if not already in closed position at the start of the code.

        // Run until the end of the match (driver presses STOP).
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Toggle control mode on left joystick button press.
            if (gamepad1.left_stick_button) {
                isFieldCentric = !isFieldCentric; // Toggle the mode.
                last_button = "left stick button"; // Sets last button to "left stick button".
                sleep(200); // Small delay to avoid multiple toggles.

                if (isFieldCentric) { // Activates when the mode is Field Centric.
                    telemetry.addData("Mode", "Field-Centric"); // Report the mode change to Field-Centric on Driver Hub.
                } else { // Activates when the mode is Field Centric.
                    telemetry.addData("Mode", "Robot-Centric"); // Report the mode change to Robot-Centric on Driver Hub.
                }

                telemetry.update(); // Adds the mode telemetry to REV Driver Hub.
            }

            // Get raw values from the gamepad.
            double y = -gamepad1.left_stick_y; // Negative because the gamepad's y-axis is inverted.
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing.
            double rx = gamepad1.right_stick_x;

            // Creates three variables that are used for the Mecanum wheel calculations.
            double forward, sideways, rotation;

            // Convert the raw x and y values to robot-centric forward and sideways velocities for easier understanding.
            forward = y;
            sideways = x;
            rotation = rx; // Set to right stick rotation value.

            // Use the LT value as an acceleration factor for all mecanum wheel movement.
            // LT value is between 0.15 (not pressed) and 1 (fully pressed).
            double lt = gamepad1.left_trigger;
            double ltSpeed = accelerationFactor + (1 - accelerationFactor) * lt;

            // Reset the yaw angle to 0 degrees when the "Back" button is pressed. Is used for Field-Centric mode, but can be activated during Robot-Centric Mode for Field-Centric mode.
            if (gamepad1.start) {
                imu.resetYaw();
                last_button = "start"; // Sets last button to "back".
            }

            // Launches the drone by moving the servo to launch position and then back to hold position.
            if (gamepad1.back) {
                droneLauncher.setPosition(launch_position); // Sets the servo to launch position, launching the drone.
                TimeUnit.SECONDS.sleep(1);
                droneLauncher.setPosition(hold_position); // Sets the servo back to hold position.
                last_button = "back"; // Sets last button to "back".
            }

            if (gamepad1.left_stick_button) {
                // Resets all encoder tick positions.
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            // Raises armMotor fully.
            if (gamepad1.a) {
                arm_speed = 0.5;

                target_arm_position = 190;
                armMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
                armMotor.setPower(arm_speed); // This sets the speed at which the arm motor will run at.
                armMotor.setTargetPosition(target_arm_position); // This sets the target position of the arm motor to target_arm_position.
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // This moves the arm motor to the target position set above.

                last_button = "a"; // Sets last button to "a".

            }

            // Closes armMotor, trapping pixels inside.
            if (gamepad1.b) {
                arm_speed = 0.5;
                target_arm_position = -10;
                armMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
                armMotor.setPower(arm_speed); // This sets the speed at which the arm motor will run at.
                armMotor.setTargetPosition(target_arm_position); // This sets the target position of the arm motor to target_arm_position.
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // This moves the arm motor to the target position set above.
                last_button = "b"; // Sets last button to "b".
            }


            // Stops arm motor if it is within 20 encoder ticks of its desired position (may need to be configured).
            if (armMotor.getCurrentPosition() <= target_arm_position + 10 && armMotor.getCurrentPosition() >= target_arm_position - 10) {
                arm_speed = 0;
            }

            /*

            // While left bumper is held, inScoWheel will turn inwards, which causes intake.
            if (gamepad2.left_bumper){
                inScoWheel.setPower(wheel_speed); // Turns inScoWheel forwards.
                last_button = "left bumper"; // Sets last button to "left bumper".

                // While right bumper is held, inScoWheel will turn outwards, which causes outtake.
            } else if (gamepad2.right_bumper) {
                inScoWheel.setPower((-wheel_speed)); // Turns inScoWheel backwards.
                last_button = "right bumper"; // Sets last button to "right bumper".

                // If neither bumpers or pressed, don't move the inScoWheel, which sets it to brake.
            } else {
                inScoWheel.setPower(0);
            }

            // Opens back flap is closed, and closes back flap if opened.
            if (gamepad2.start) {
                last_button = "start"; // Sets last button to "start".
                sleep(200); // Small delay to avoid multiple toggles.

                if (backFlapFlag) {
                    backFlap.setPosition(0.5); // Open back flap.
                } else {
                    backFlap.setPosition(0); // Close back flap.
                }
                backFlapFlag = !backFlapFlag; // Inverts true/false boolean trigger flag.
            }

            */

            // If the robot is in Field-Centric Mode, the robot will NOT have a head (meaning that the robot's controls WILL NOT change based off the direction it is facing).
            // What direction is forward can be done be resetting the yaw angle to 0 degrees (through pressing gamepad.back).
            if (isFieldCentric) {

                // Calculate motor powers using mecanum drive kinematics.
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the robot's rotation.
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                // Denominator is the largest motor power (absolute value) or 1.
                // This ensures all the powers maintain the same ratio, but only when at least one is out of the range [-1, 1].
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rotation) / denominator * ltSpeed;
                double backLeftPower = (rotY - rotX + rotation) / denominator * ltSpeed;
                double frontRightPower = (rotY - rotX - rotation) / denominator * ltSpeed;
                double backRightPower = (rotY + rotX - rotation) / denominator * ltSpeed;

                // Set motor powers.
                motorFrontLeft.setPower(-frontLeftPower);
                motorBackLeft.setPower(-backLeftPower);
                motorFrontRight.setPower(-frontRightPower);
                motorBackRight.setPower(-backRightPower);

                telemetry.addData("Mode:", "Field-Centric"); // Displays current mode (Field-Centric).
                telemetry.addData("Front Left Power", frontLeftPower); // Displays power of the front left mecanum wheel.
                telemetry.addData("Back Left Power", backLeftPower); // Displays power of the back left mecanum wheel.
                telemetry.addData("Front Right Power", frontRightPower); // Displays power of the front right mecanum wheel.
                telemetry.addData("Back Right Power", backRightPower); // Displays power of the back right mecanum wheel.
            }

            // If the robot is in Robot-Centric Mode, the robot will WILL have a head (meaning that the robot's controls WILL change based off the direction it is facing).
            // You can still reset the yaw angle to 0 by using the back button in Robot-Centric mode.
            else {
                // Calculate motor powers using mecanum drive kinematics.
                // No denominator is needed here in Robot-Centric Mode.
                double frontLeftPower = (forward + sideways + rotation) * ltSpeed;
                double frontRightPower = (forward - sideways - rotation) * ltSpeed;
                double backLeftPower = (forward - sideways + rotation) * ltSpeed;
                double backRightPower = (forward + sideways - rotation) * ltSpeed;

                // Set motor powers.
                motorFrontLeft.setPower(-frontLeftPower);
                motorBackLeft.setPower(-backLeftPower);
                motorFrontRight.setPower(-frontRightPower);
                motorBackRight.setPower(-backRightPower);

                telemetry.addData("Mode:", "Robot-Centric"); // Displays current mode (Robot-Centric).
                telemetry.addData("Front Left Power", frontLeftPower); // Displays power of the front left mecanum wheel.
                telemetry.addData("Back Left Power", backLeftPower); // Displays power of the back left mecanum wheel.
                telemetry.addData("Front Right Power", frontRightPower); // Displays power of the front right mecanum wheel.
                telemetry.addData("Back Right Power", backRightPower); // Displays power of the back right mecanum wheel.
            }
            telemetry.addData("Speed (Left Trigger)", ltSpeed); // Displays speed of robot mecanum wheel movement using the left trigger (between 0.15 and 1).
            telemetry.addData("Last button pressed", last_button); // Displays the last gamepad 1 press/call (excluding joystick movement).
            telemetry.addData("motorFrontLeft Position", motorFrontLeft.getCurrentPosition());
            telemetry.addData("motorBackLeft Position", motorBackLeft.getCurrentPosition());
            telemetry.addData("motorFrontRight Position", motorFrontRight.getCurrentPosition());
            telemetry.addData("motorBackRight Position", motorBackRight.getCurrentPosition());
            telemetry.update();
            telemetry.update(); // Adds telemetry to REV Driver Hub.
        }
    }
}
