package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "ButtonRPM", group = "TeleOp")
public class ButtonRPM extends LinearOpMode {

    private static final String SHOOTER_NAME = "shooter";
    private static final double TARGET_RPM = 2200.0;

    // 28 motor ticks/rev × 3/4 motor rev per flywheel rev.
    private static final double TICKS_PER_FLYWHEEL_REV = 28.0 * (3.0 / 4.0);

    // Custom velocity PIDF coefficients. Tune for the specific motor, hub, and shooter.
    private static final double PID_P = 0.0015;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;
    private static final double PID_F = 0.005;

    @Override
    public void runOpMode() {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);

        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply custom velocity PIDF values after enabling RUN_USING_ENCODER.
        shooter.setVelocityPIDFCoefficients(PID_P, PID_I, PID_D, PID_F);

        telemetry.addLine("Press Driver Station START, then gamepad A to toggle shooter.");
        telemetry.addData("Custom PIDF (P/I/D/F)", "%.4f / %.4f / %.4f / %.4f",
                PID_P, PID_I, PID_D, PID_F);
        telemetry.update();

        waitForStart();

        boolean shooterEnabled = false;
        boolean previousA = false;
        boolean previousStart = false;

        while (opModeIsActive()) {
            boolean a = gamepad1.a;
            boolean start = gamepad1.start;

            // A starts the shooter; gamepad Start stops it.
            if (a && !previousA) {
                shooterEnabled = true;
            }
            if (start && !previousStart) {
                shooterEnabled = false;
            }

            previousA = a;
            previousStart = start;

            double targetRpm = shooterEnabled ? TARGET_RPM : 0.0;
            double targetTicksPerSecond =
                    targetRpm * TICKS_PER_FLYWHEEL_REV / 60.0;

            shooter.setVelocity(targetTicksPerSecond);

            telemetry.addData("Shooter enabled", shooterEnabled);
            telemetry.addData("Target RPM", "%.0f", targetRpm);
            telemetry.update();

            idle();
        }

        shooter.setVelocity(0.0);
    }
}
