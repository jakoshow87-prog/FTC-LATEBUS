package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//will rewrite soon with PIDF control
@TeleOp(name = "Shooter OpMode", group = "Test")
public class ShooterTestOpMode extends LinearOpMode {

    private DcMotor shooterMotor;

    @Override
    public void runOpMode() {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter");

        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                shooterMotor.setPower(0.85);
            }
            else if (gamepad1.start) {
                shooterMotor.setPower(0.0);
            }

            // Show data on the Driver Station
            telemetry.addData("Shooter Power", shooterMotor.getPower());
            telemetry.update();
        }
    }
}


