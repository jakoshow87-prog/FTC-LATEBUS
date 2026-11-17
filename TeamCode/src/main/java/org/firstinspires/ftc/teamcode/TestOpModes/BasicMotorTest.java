package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// Motor test
@TeleOp(name = "BasicMotorTest", group = "Test")
public class BasicMotorTest extends LinearOpMode {

    private DcMotor Motor;

    @Override
    public void runOpMode() {
        Motor = hardwareMap.get(DcMotor.class, "motor");
        Motor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.start) {
                Motor.setPower(0.0);
            } else if (gamepad1.a) {
                Motor.setPower(0.1);
            } else if (gamepad1.b) {
                Motor.setPower(0.75);
            } else if (gamepad1.x) {
                Motor.setPower(0.50);
            } else if (gamepad1.y) {
                Motor.setPower(0.25);
            }

            telemetry.addData("Motor Power",Motor.getPower());
            telemetry.update();
        }
    }
}