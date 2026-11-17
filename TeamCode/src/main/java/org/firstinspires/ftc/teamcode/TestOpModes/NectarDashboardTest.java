package org.firstinspires.ftc.teamcode.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 *  first connect to control hub's wifi
 *  then go to 192.168.43.1:8080/dash or 192.168.49.1:8080/dash in web browser
 *  now you should be able to use dashboard
 */
@Config
@TeleOp(name = "NectarDashboardTest", group = "Test")
public class NectarDashboardTest extends LinearOpMode {

    private static final String SHOOTER_NAME = "Nectar";

    private static final double TARGET_RPM = 2200.0;
    private static final double RPM_STEP = 100.0;
    private static final double RPM_TOLERANCE = 50.0;

    // Encoder resolution at the motor shaft.
    private static final double MOTOR_TICKS_PER_REV = 28.0;

    // Three motor revolutions produce four flywheel revolutions.
    private static final double MOTOR_REVS_PER_FLYWHEEL_REV = 3.0 / 4.0;

    private static final double TICKS_PER_FLYWHEEL_REV =
            MOTOR_TICKS_PER_REV * MOTOR_REVS_PER_FLYWHEEL_REV;

    // Edit these live from FTC Dashboard.
    public static double PID_P = 0.0022;
    public static double PID_I = 0.0;
    public static double PID_D = 0.0002;
    public static double PID_F = 42.56;

    @Override
    public void runOpMode() {
        DcMotorEx shooter =
                hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);

        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Show telemetry on both the Driver Station and Dashboard.
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        double targetRpm = 0.0;

        telemetry.addLine("Nectar Shooter PIDF Tuner");
        telemetry.addLine("A: Start/stop shooter");
        telemetry.addLine("D-pad up/down: Adjust target RPM");
        telemetry.addLine("B: Set target to 2200 RPM");
        telemetry.addLine("X: Stop shooter");
        telemetry.update();

        waitForStart();

        boolean shooterEnabled = false;
        boolean previousA = false;
        boolean previousB = false;
        boolean previousX = false;
        boolean previousUp = false;
        boolean previousDown = false;

        // Remember which PIDF values are currently applied.
        double appliedP = Double.NaN;
        double appliedI = Double.NaN;
        double appliedD = Double.NaN;
        double appliedF = Double.NaN;

        while (opModeIsActive()) {
            // Apply Dashboard PIDF edits when a value changes.
            if (Double.compare(PID_P, appliedP) != 0
                    || Double.compare(PID_I, appliedI) != 0
                    || Double.compare(PID_D, appliedD) != 0
                    || Double.compare(PID_F, appliedF) != 0) {

                shooter.setVelocityPIDFCoefficients(
                        PID_P, PID_I, PID_D, PID_F
                );

                appliedP = PID_P;
                appliedI = PID_I;
                appliedD = PID_D;
                appliedF = PID_F;
            }

            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            if (a && !previousA) {
                shooterEnabled = !shooterEnabled;
                if (!shooterEnabled) {
                    targetRpm = 0.0;
                    shooter.setVelocity(0.0);
                }
            }

            if (b && !previousB) {
                targetRpm = TARGET_RPM;
                shooterEnabled = true;
            }

            if (x && !previousX) {
                targetRpm = 0.0;
                shooterEnabled = false;
                shooter.setVelocity(0.0);
            }

            if (up && !previousUp) {
                targetRpm = Math.min(5000.0, targetRpm + RPM_STEP);
                shooterEnabled = targetRpm > 0.0;
            }

            if (down && !previousDown) {
                targetRpm = Math.max(0.0, targetRpm - RPM_STEP);
                if (targetRpm == 0.0) {
                    shooterEnabled = false;
                }
            }

            previousA = a;
            previousB = b;
            previousX = x;
            previousUp = up;
            previousDown = down;

            double targetTicksPerSecond =
                    targetRpm * TICKS_PER_FLYWHEEL_REV / 60.0;

            shooter.setVelocity(
                    shooterEnabled && targetRpm > 0.0
                            ? targetTicksPerSecond
                            : 0.0
            );

            double measuredVelocity = shooter.getVelocity();
            double measuredRpm =
                    measuredVelocity * 60.0 / TICKS_PER_FLYWHEEL_REV;
            double errorRpm = targetRpm - measuredRpm;

            telemetry.addData("Shooter enabled", shooterEnabled);
            telemetry.addData("Target RPM", "%.0f", targetRpm);
            telemetry.addData("Measured RPM", "%.0f", measuredRpm);
            telemetry.addData("RPM error", "%.0f", errorRpm);
            telemetry.addData("At target speed",
                    shooterEnabled
                            && Math.abs(errorRpm) <= RPM_TOLERANCE);
            telemetry.addData("Motor velocity (ticks/sec)", "%.1f",
                    measuredVelocity);
            telemetry.addData("PIDF", "P %.6f | I %.6f | D %.6f | F %.6f",
                    PID_P, PID_I, PID_D, PID_F);
            telemetry.update();

            idle();
        }

        shooter.setVelocity(0.0);
    }
}