package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "ForwardDrive", group = "Auto")
public class ForwardDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {
            // Drive forward for 5 seconds
            ElapsedTime timer = new ElapsedTime();
            timer.reset();

            frontLeft.setPower(0.65);
            frontRight.setPower(0.65);
            backLeft.setPower(0.65);
            backRight.setPower(0.65);

            while (opModeIsActive() && timer.seconds() < 1.0) {
            }

            // Stop briefly before rotating
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            sleep(500);

            // Rotate for 1 second (adjust duration and power to change the turn)
            timer.reset();
            frontLeft.setPower(-0.65);
            frontRight.setPower(0.65);
            backLeft.setPower(-0.65);
            backRight.setPower(0.65);

            while (opModeIsActive() && timer.seconds() < 1.0) {
            }

            // Stop after rotating
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

    }