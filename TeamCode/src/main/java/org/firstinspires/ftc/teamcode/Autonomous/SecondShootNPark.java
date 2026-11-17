package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "SecondShootNPark", group = "Auto")
public class SecondShootNPark extends LinearOpMode {

    private static final String POLLEN_SHOOTER_NAME = "Pollen";
    private static final String POLLEN_GATE_SERVO_NAME = "Gate";

    private static final double POLLEN_GATE_CENTER = 0.77;
    private static final double POLLEN_GATE_RIGHT = 0.85;

    private static final double POLLEN_TARGET_RPM = 2500.0;
    private static final double POLLEN_TICKS_PER_REV = 28.0;

    public static double POLLEN_PID_P = 0.0016;
    public static double POLLEN_PID_I = 0.0;
    public static double POLLEN_PID_D = 0.2;
    public static double POLLEN_PID_F = 14.5;

    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        DcMotorEx pollenShooter =
                hardwareMap.get(DcMotorEx.class, POLLEN_SHOOTER_NAME);
        Servo pollenGate =
                hardwareMap.get(Servo.class, POLLEN_GATE_SERVO_NAME);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        pollenShooter.setDirection(DcMotor.Direction.REVERSE);
        pollenShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pollenShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenShooter.setVelocityPIDFCoefficients(
                POLLEN_PID_P,
                POLLEN_PID_I,
                POLLEN_PID_D,
                POLLEN_PID_F
        );
        pollenShooter.setVelocity(0.0);
        pollenGate.setPosition(POLLEN_GATE_CENTER);

        waitForStart();

        if (opModeIsActive()) {
            // Start the pollen shooter and let it spin up for 2 seconds.
            double pollenTargetTicksPerSecond =
                    POLLEN_TARGET_RPM * POLLEN_TICKS_PER_REV / 60.0;
            pollenShooter.setVelocity(pollenTargetTicksPerSecond);
            sleep(2000);

            // Cycle the gate for 5 seconds while the pollen shooter runs.
            if (opModeIsActive()) {
                boolean gateRight = false;
                long gateStartNanos = System.nanoTime();
                long gateEndNanos = gateStartNanos + 5_000_000_000L;
                long nextGateSwitchNanos = gateStartNanos + 1_000_000_000L;

                while (opModeIsActive()
                        && System.nanoTime() < gateEndNanos) {
                    long now = System.nanoTime();

                    if (now >= nextGateSwitchNanos) {
                        gateRight = !gateRight;
                        pollenGate.setPosition(
                                gateRight
                                        ? POLLEN_GATE_RIGHT
                                        : POLLEN_GATE_CENTER
                        );
                        nextGateSwitchNanos += 1_000_000_000L;
                    }

                    idle();
                }
            }

            // Stop the shooter, center the gate, and wait before driving.
            pollenShooter.setVelocity(0.0);
            pollenGate.setPosition(POLLEN_GATE_CENTER);
            sleep(3000);

            if (opModeIsActive()) {
                // Drive forward for 1 second.
                frontLeft.setPower(0.65);
                frontRight.setPower(0.65);
                backLeft.setPower(0.65);
                backRight.setPower(0.65);
                sleep(500);

                stopMotors(frontLeft, frontRight, backLeft, backRight);
                sleep(500);

                if (opModeIsActive()) {
                    // Left Rotation
                    frontLeft.setPower(-0.65);
                    frontRight.setPower(0.65);
                    backLeft.setPower(-0.65);
                    backRight.setPower(0.65);
                    sleep(1150);
                }

                stopMotors(frontLeft, frontRight, backLeft, backRight);
                sleep(500);

                if (opModeIsActive()) {
                    // Right Rotation
                    frontLeft.setPower(0.65);
                    frontRight.setPower(-0.65);
                    backLeft.setPower(0.65);
                    backRight.setPower(-0.65);
                    sleep(1150);

                    // Foward drive until aligned with Park
                    if (opModeIsActive()) {
                        frontLeft.setPower(0.65);
                        frontRight.setPower(0.65);
                        backLeft.setPower(0.65);
                        backRight.setPower(0.65);
                        sleep(4000);

                        if (opModeIsActive()) {
                            // Left Rotation
                            frontLeft.setPower(-0.65);
                            frontRight.setPower(0.65);
                            backLeft.setPower(-0.65);
                            backRight.setPower(0.65);
                            sleep(1150);

                            if (opModeIsActive()) {
                                // Right Rotation
                                frontLeft.setPower(0.65);
                                frontRight.setPower(0.65);
                                backLeft.setPower(0.65);
                                backRight.setPower(0.65);
                                sleep(1150);
                            }
                        }

                    }
                }
            }
        }

        // Stop and reset mechanisms when the auto finishes or is interrupted.
        stopMotors(frontLeft, frontRight, backLeft, backRight);
        pollenShooter.setVelocity(0.0);
        pollenGate.setPosition(POLLEN_GATE_CENTER);
    }

    private void stopMotors(
            DcMotor frontLeft,
            DcMotor frontRight,
            DcMotor backLeft,
            DcMotor backRight) {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }
}