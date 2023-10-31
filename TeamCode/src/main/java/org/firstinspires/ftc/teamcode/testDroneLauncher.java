package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@Config

@TeleOp
public class testDroneLauncher extends LinearOpMode {

    private Servo testDroneLauncher;

    @Override
    public void runOpMode() throws InterruptedException {
        testDroneLauncher = hardwareMap.get(Servo.class, "testDroneLauncher");
        testDroneLauncher.setDirection(Servo.Direction.FORWARD); // Set the servo direction only once

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        testDroneLauncher.setPosition(0.6);


        waitForStart();

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y;
            double x = gamepad1.right_stick_x;

            telemetry.addData("X = ", x);
            telemetry.addData("Y = ", y);
            telemetry.update();

            if (gamepad1.a) {
                telemetry.addData("Status", "a - pressed");
                testDroneLauncher.setPosition(0.6); // Set to the maximum position
            }

            if (gamepad1.b) {
                telemetry.addData("Status", "b - pressed");
                testDroneLauncher.setPosition(0.0); // Set to the desired minimum position
            }

            // Update telemetry
            telemetry.update();

        }
    }
}