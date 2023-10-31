package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

@TeleOp
public class Front_Cage extends LinearOpMode{

    DcMotor Front_Cage;
    @Override
    public void runOpMode() throws InterruptedException{
        Front_Cage = hardwareMap.dcMotor.get("Front_Cage");
        Front_Cage.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_Cage.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        double front_cage_motor_speed = 1;
        int openEncoderValue = 100;
        int closeEncoderValue = -100;
        waitForStart();
        telemetry.addData("Status", "Initialized");
        if (isStopRequested()) return;
        while (opModeIsActive()){
            if (gamepad1.a){
                Front_Cage.setPower(front_cage_motor_speed);
                Front_Cage.setTargetPosition(openEncoderValue);
                telemetry.addData("Status", "Open Position");
            } else if (gamepad1.b) {
                Front_Cage.setPower(front_cage_motor_speed);
                Front_Cage.setTargetPosition(closeEncoderValue);
                telemetry.addData("Status", "Closed Position");
            }
        }
    }
}
