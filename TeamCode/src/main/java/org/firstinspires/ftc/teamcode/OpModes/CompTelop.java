
package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp(name = "CompTelop", group = "MAINOPMODES")
public class CompTelop extends LinearOpMode {

    private static final String INTAKE_MOTOR_NAME = "Intake";
    private static final String SHOOTER_NAME = "Nectar";
    private static final String POLLEN_SHOOTER_NAME = "Pollen";
    private static final String KICKER_SERVO_NAME = "kicker";
    private static final String POLLEN_GATE_SERVO_NAME = "Gate";

    // GATE SETTINGS
    private static final double POLLEN_GATE_CENTER = 0.77;
    private static final double POLLEN_GATE_RIGHT = 0.85;
    private static final long GATE_DWELL_START_MS = 1000L;
    private static final long GATE_DWELL_INCREMENT_MS = 500L;
    private static final long GATE_DWELL_ADJUST_MS = 250L;
    private static final long GATE_DWELL_MIN_MS = 500L;
    private static final long GATE_DWELL_MAX_MS = 10000L;

    // NECTAR SETTINGS
    private static final double WALL_SHOT = 2200.0;
    private static final double TICKS_PER_FLYWHEEL_REV =
            28.0 * (3.0 / 4.0);

    // POLLEN SETTINGS
    private static final double POLLEN_TARGET_RPM = 2500.0;
    private static final double POLLEN_TICKS_PER_FLYWHEEL_REV = 28.0;

    // NECTAR PIDF
    private static final double NECTAR_PID_P = 0.0018;
    private static final double NECTAR_PID_I = 0.0;
    private static final double NECTAR_PID_D = 0.0;

    private static final double NECTAR_PID_F = 42.56;

    // POLLEN PIDF
    private static final double POLLEN_PID_P = 0.15;
    private static final double POLLEN_PID_I = 0.0;
    private static final double POLLEN_PID_D = 0.0006;

    // Automatically recalculates when target RPM changes.
    private static final double POLLEN_PID_F = 15;

    private static final double KICKER_POWER = -1.0;

    @Override
    public void runOpMode() {

        // DRIVETRAIN
        DcMotor frontLeftMotor =
                hardwareMap.get(DcMotor.class, "frontLeft");

        DcMotor frontRightMotor =
                hardwareMap.get(DcMotor.class, "frontRight");

        DcMotor backLeftMotor =
                hardwareMap.get(DcMotor.class, "backLeft");

        DcMotor backRightMotor =
                hardwareMap.get(DcMotor.class, "backRight");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // SUBSYSTEMS
        Intake intake = new Intake(hardwareMap, INTAKE_MOTOR_NAME);

        DcMotorEx shooter =
                hardwareMap.get(DcMotorEx.class, SHOOTER_NAME);

        DcMotorEx pollenShooter =
                hardwareMap.get(DcMotorEx.class, POLLEN_SHOOTER_NAME);

        CRServo kickerServo =
                hardwareMap.get(CRServo.class, KICKER_SERVO_NAME);

        Servo pollenGate =
                hardwareMap.get(Servo.class, POLLEN_GATE_SERVO_NAME);

        VoltageSensor battery =
                hardwareMap.voltageSensor.iterator().next();

        // NECTAR INITIALIZATION
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setVelocityPIDFCoefficients(
                NECTAR_PID_P,
                NECTAR_PID_I,
                NECTAR_PID_D,
                NECTAR_PID_F
        );

        shooter.setVelocity(0.0);

        // POLLEN INITIALIZATION
        // Reverse direction is intentional.
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

        // SERVO INITIALIZATION
        kickerServo.setPower(0.0);
        pollenGate.setPosition(POLLEN_GATE_CENTER);

        telemetry.addLine("COMP TELEOP READY");
        telemetry.addLine("A: Nectar shooter");
        telemetry.addLine("B: Toggle Pollen shooter");
        telemetry.addLine("LB: Toggle kicker");
        telemetry.addLine("RB: Start/stop pollen gate loop" );
        telemetry.addLine("D-pad up/down: adjust starting gate hold time" );
        telemetry.addLine("START: Stop shooters/kicker; return gate to center");
        telemetry.update();

        waitForStart();

        boolean nectarEnabled = false;
        boolean pollenEnabled = false;

        boolean previousA = false;
        boolean previousPollenButton = false;

        boolean kickerEnabled = false;
        boolean previousLB = false;

        boolean previousStart = false;

        boolean gateLoopEnabled = false;
        boolean pollenGateRight = false;
        long gateBaseDwellMs = GATE_DWELL_START_MS;
        long gateSwitchCount = 0;
        long gateNextSwitchNanos = 0L;
        boolean previousRB = false;
        boolean previousDpadUp = false;
        boolean previousDpadDown = false;

        while (opModeIsActive()) {

            // DRIVETRAIN
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double frontLeftPower = y + x + rx;
            double frontRightPower = y - x - rx;
            double backLeftPower = y - x + rx;
            double backRightPower = y + x - rx;

            double max = Math.max(1.0, Math.max(
                    Math.max(Math.abs(frontLeftPower),
                            Math.abs(backLeftPower)),
                    Math.max(Math.abs(frontRightPower),
                            Math.abs(backRightPower))
            ));

            frontLeftMotor.setPower(frontLeftPower / max);
            frontRightMotor.setPower(frontRightPower / max);
            backLeftMotor.setPower(backLeftPower / max);
            backRightMotor.setPower(backRightPower / max);

            // INTAKE
            if (gamepad1.right_trigger > 0.1) {
                intake.run();
            } else if (gamepad1.left_trigger > 0.1) {
                intake.reverse();
            } else {
                intake.stop();
            }

            // BUTTON INPUTS
            boolean a = gamepad1.a;
            boolean pollenButton = gamepad1.b;
            boolean lb = gamepad1.left_bumper;
            boolean start = gamepad1.start;
            boolean rb = gamepad1.right_bumper;
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;

            if (a && !previousA && !nectarEnabled) {
                nectarEnabled = true;
            }

            if (pollenButton && !previousPollenButton) {
                pollenEnabled = !pollenEnabled;
            }

            if (lb && !previousLB) {
                kickerEnabled = !kickerEnabled;
            }

            if (rb && !previousRB) {
                gateLoopEnabled = !gateLoopEnabled;
                if (gateLoopEnabled) {
                    gateNextSwitchNanos = System.nanoTime() + gateBaseDwellMs * 1_000_000L;
                }
            }

            if (dpadUp && !previousDpadUp) {
                gateBaseDwellMs = Math.min(GATE_DWELL_MAX_MS, gateBaseDwellMs + GATE_DWELL_ADJUST_MS);
            }
            if (dpadDown && !previousDpadDown) {
                gateBaseDwellMs = Math.max(GATE_DWELL_MIN_MS, gateBaseDwellMs - GATE_DWELL_ADJUST_MS);
            }

            if (start && !previousStart) {
                nectarEnabled = false;
                pollenEnabled = false;
                kickerEnabled = false;
                gateLoopEnabled = false;
                pollenGateRight = false;
                gateBaseDwellMs = GATE_DWELL_START_MS;
                gateSwitchCount = 0;
                pollenGate.setPosition(POLLEN_GATE_CENTER);
            }

            previousA = a;
            previousPollenButton = pollenButton;
            previousLB = lb;
            previousStart = start;
            previousRB = rb;
            previousDpadUp = dpadUp;
            previousDpadDown = dpadDown;

            // TARGET VELOCITIES
            double nectarTicksPerSecond =
                    WALL_SHOT * TICKS_PER_FLYWHEEL_REV / 60.0;

            double pollenTicksPerSecond =
                    POLLEN_TARGET_RPM
                            * POLLEN_TICKS_PER_FLYWHEEL_REV / 60.0;

            shooter.setVelocity(
                    nectarEnabled ? nectarTicksPerSecond : 0.0
            );

            pollenShooter.setVelocity(
                    pollenEnabled ? pollenTicksPerSecond : 0.0
            );

            // SERVOS
            kickerServo.setPower(
                    kickerEnabled ? KICKER_POWER : 0.0
            );

            if (gateLoopEnabled && System.nanoTime() >= gateNextSwitchNanos) {
                pollenGateRight = !pollenGateRight;
                pollenGate.setPosition(pollenGateRight ? POLLEN_GATE_RIGHT : POLLEN_GATE_CENTER);
                gateSwitchCount++;
                long dwellMs = gateBaseDwellMs + (gateSwitchCount / 2) * GATE_DWELL_INCREMENT_MS;
                gateNextSwitchNanos = System.nanoTime() + dwellMs * 1_000_000L;
            }

            // RAW ENCODER VELOCITIES
            double nectarRawVelocity = shooter.getVelocity();
            double pollenRawVelocity = pollenShooter.getVelocity();

            // CALCULATED RPM
            double nectarMeasuredRpm =
                    nectarRawVelocity * 60.0
                            / TICKS_PER_FLYWHEEL_REV;

            double pollenMeasuredRpm =
                    pollenRawVelocity * 60.0
                            / POLLEN_TICKS_PER_FLYWHEEL_REV;

            // ACTIVE PIDF
            PIDFCoefficients nectarPIDF =
                    shooter.getPIDFCoefficients(
                            DcMotor.RunMode.RUN_USING_ENCODER
                    );

            PIDFCoefficients pollenPIDF =
                    pollenShooter.getPIDFCoefficients(
                            DcMotor.RunMode.RUN_USING_ENCODER
                    );

            // NECTAR TELEMETRY
            telemetry.addLine("========== NECTAR ==========");
            telemetry.addData("Enabled", nectarEnabled);
            telemetry.addData("Target RPM", "%.0f",
                    nectarEnabled ? WALL_SHOT : 0.0);
            telemetry.addData("Target Ticks/sec", "%.2f",
                    nectarTicksPerSecond);
            telemetry.addData("Raw Velocity", "%.2f",
                    nectarRawVelocity);
            telemetry.addData("Measured RPM", "%.0f",
                    nectarMeasuredRpm);
            telemetry.addData("Motor Power", "%.3f",
                    shooter.getPower());

            // NECTAR PIDF
            telemetry.addData("Nectar PIDF",
                    "%.6f / %.6f / %.6f / %.6f",
                    nectarPIDF.p,
                    nectarPIDF.i,
                    nectarPIDF.d,
                    nectarPIDF.f);

            // POLLEN TELEMETRY
            telemetry.addLine("========== POLLEN ==========");
            telemetry.addData("Enabled", pollenEnabled);
            telemetry.addData("Target RPM", "%.0f",
                    pollenEnabled ? POLLEN_TARGET_RPM : 0.0);
            telemetry.addData("Target Ticks/sec", "%.2f",
                    pollenTicksPerSecond);
            telemetry.addData("Raw Velocity", "%.2f",
                    pollenRawVelocity);
            telemetry.addData("Calculated RPM", "%.0f",
                    pollenMeasuredRpm);
            telemetry.addData("Motor Power", "%.3f",
                    pollenShooter.getPower());
            telemetry.addData("Direction", "REVERSE");

            // POLLEN PIDF
            telemetry.addData("Pollen PIDF",
                    "%.6f / %.6f / %.6f / %.6f",
                    pollenPIDF.p,
                    pollenPIDF.i,
                    pollenPIDF.d,
                    pollenPIDF.f);

            // OTHER TELEMETRY
            telemetry.addLine("========== OTHER ==========");
            telemetry.addData("Pollen Gate loop", gateLoopEnabled ? "Running" : "Stopped");
            telemetry.addData("Gate position", "%.2f", pollenGateRight ? POLLEN_GATE_RIGHT : POLLEN_GATE_CENTER);
            telemetry.addData("Gate dwell", "%.2f sec", (gateBaseDwellMs + (gateSwitchCount / 2) * GATE_DWELL_INCREMENT_MS) / 1000.0);

            telemetry.addData("Kicker",
                    kickerEnabled ? "Running" : "Stopped");

            telemetry.addData("Intake",
                    gamepad1.right_trigger > 0.1 ? "Forward"
                            : gamepad1.left_trigger > 0.1 ? "Reverse"
                              : "Stopped");

            telemetry.addData("Battery Voltage", "%.2f V",
                    battery.getVoltage());

            telemetry.update();
            idle();
        }

        // STOP EVERYTHING
        intake.stop();

        shooter.setVelocity(0.0);
        pollenShooter.setVelocity(0.0);

        kickerServo.setPower(0.0);

        pollenGate.setPosition(POLLEN_GATE_CENTER);

        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
    }
}







