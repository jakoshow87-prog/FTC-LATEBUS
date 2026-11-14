package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "CornerPark", group = "Auto")
public class CornerPark extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {
            // Drive forward for 1 second
            frontLeft.setPower(0.65);
            frontRight.setPower(0.65);
            backLeft.setPower(0.65);
            backRight.setPower(0.65);
            sleep(1000);

            stopMotors(frontLeft, frontRight, backLeft, backRight);
            sleep(500);
        }
    }

    private void stopMotors(
            DcMotor frontLeft,
            DcMotor frontRight,
            DcMotor backLeft,
            DcMotor backRight) {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}

