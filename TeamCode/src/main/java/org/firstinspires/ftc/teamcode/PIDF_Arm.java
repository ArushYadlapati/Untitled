package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "SingleJointedArmTeleOp", group = "TeleOp")
public class PIDF_Arm extends LinearOpMode {

    private DcMotor lowerJoint;
    private DcMotor upperJoint;

    private PIDFController lowerJointPIDF;
    private PIDFController upperJointPIDF;

    private double accelerationFactor = 0.1; // Adjust as needed
    /*** TUNING VARIABLES: ***/
    private double kP = 1.0;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kF = 0.0;

    @Override
    public void runOpMode() {
        lowerJoint = hardwareMap.dcMotor.get("lowerJoint");
        upperJoint = hardwareMap.dcMotor.get("upperJoint");

        lowerJoint.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperJoint.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lowerJoint.setDirection(DcMotorSimple.Direction.REVERSE); // Adjust direction as needed

        lowerJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lowerJointPIDF = new PIDFController(kP, kI, kD, kF);
        upperJointPIDF = new PIDFController(kP, kI, kD, kF);

        waitForStart();

        while (opModeIsActive()) {
            double lt = gamepad1.left_trigger;
            double ltSpeed = accelerationFactor + (1 - accelerationFactor) * lt;

            // Get joystick inputs for arm control.
            double lowerJointPower = -gamepad1.left_stick_y;
            double upperJointPower = -gamepad1.right_stick_y;

            // Calculate PIDF outputs.
            double lowerJointOutput = lowerJointPIDF.calculate(lowerJoint.getCurrentPosition(), 0); // Replace '0' with desired setpoint
            double upperJointOutput = upperJointPIDF.calculate(upperJoint.getCurrentPosition(), 0); // Replace '0' with desired setpoint

            // Set proportional speed using ltSpeed.
            lowerJointOutput *= ltSpeed;
            upperJointOutput *= ltSpeed;

            // Set power to motors based on PIDF outputs.
            lowerJoint.setPower(lowerJointOutput);
            upperJoint.setPower(upperJointOutput);

            telemetry.addData("Lower Joint Position", lowerJoint.getCurrentPosition());
            telemetry.addData("Upper Joint Position", upperJoint.getCurrentPosition());
            telemetry.addData("Lower Joint Output", lowerJointOutput);
            telemetry.addData("Upper Joint Output", upperJointOutput);
            telemetry.addData("Proportional Speed (ltSpeed)", ltSpeed);
            telemetry.update();
        }
    }
}
