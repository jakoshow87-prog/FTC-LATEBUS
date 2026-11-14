package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    // Gate positions
    private static final double POLLEN_GATE_CENTER = 0.77;
    private static final double POLLEN_GATE_RIGHT = 0.85;

    // Nectar settings
    private static final double WALL_SHOT = 2200.0;
    private static final double TICKS_PER_FLYWHEEL_REV =
            28.0 * (3.0 / 4.0);

    // Pollen settings
    private static final double POLLEN_TARGET_RPM = 2500.0;
    private static final double POLLEN_TICKS_PER_FLYWHEEL_REV = 28.0;

    // Nectar PIDF
    private static final double NECTAR_PID_P = 0.0018;
    private static final double NECTAR_PID_I = 0.0;
    private static final double NECTAR_PID_D = 0.0;
    private static final double NECTAR_PID_F = 42.56;

    // Pollen PIDF
    private static final double POLLEN_PID_P = 0.16;
    private static final double POLLEN_PID_I = 0.0;
    private static final double POLLEN_PID_D = 0.0006;
    private static final double POLLEN_PID_F = 15.0;

    private static final double KICKER_POWER = -1.0;

    @Override
    public void runOpMode() {
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

        // Nectar initialization
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

        // Pollen initialization
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

        // Servo initialization
        kickerServo.setPower(0.0);
        pollenGate.setPosition(POLLEN_GATE_CENTER);

        telemetry.addLine("COMP TELEOP READY");
        telemetry.addLine("A: Nectar shooter");
        telemetry.addLine("B: Toggle Pollen shooter");
        telemetry.addLine("LB: Toggle kicker");
        telemetry.addLine("RB: Toggle pollen gate between center and right");
        telemetry.addLine("START: Stop shooters/kicker; return gate to center");
        telemetry.update();

        waitForStart();

        boolean nectarEnabled = false;
        boolean pollenEnabled = false;
        boolean kickerEnabled = false;
        boolean pollenGateRight = false;

        boolean previousA = false;
        boolean previousPollenButton = false;
        boolean previousLB = false;
        boolean previousRB = false;
        boolean previousStart = false;

        while (opModeIsActive()) {
            // Drivetrain
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

            // Intake
            if (gamepad1.right_trigger > 0.1) {
                intake.run();
            } else if (gamepad1.left_trigger > 0.1) {
                intake.reverse();
            } else {
                intake.stop();
            }

            // Button inputs
            boolean a = gamepad1.a;
            boolean pollenButton = gamepad1.b;
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;
            boolean start = gamepad1.start;

            if (a && !previousA && !nectarEnabled) {
                nectarEnabled = true;
            }

            if (pollenButton && !previousPollenButton) {
                pollenEnabled = !pollenEnabled;
            }

            if (lb && !previousLB) {
                kickerEnabled = !kickerEnabled;
            }

            // Each RB press changes the gate to the opposite position.
            if (rb && !previousRB) {
                pollenGateRight = !pollenGateRight;
            }

            if (start && !previousStart) {
                nectarEnabled = false;
                pollenEnabled = false;
                kickerEnabled = false;
                pollenGateRight = false;
            }

            previousA = a;
            previousPollenButton = pollenButton;
            previousLB = lb;
            previousRB = rb;
            previousStart = start;

            // Shooter target velocities
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

            // Servos
            kickerServo.setPower(kickerEnabled ? KICKER_POWER : 0.0);
            pollenGate.setPosition(
                    pollenGateRight
                            ? POLLEN_GATE_RIGHT
                            : POLLEN_GATE_CENTER
            );

            // Shooter telemetry
            double nectarRawVelocity = shooter.getVelocity();
            double pollenRawVelocity = pollenShooter.getVelocity();

            double nectarMeasuredRpm =
                    nectarRawVelocity * 60.0 / TICKS_PER_FLYWHEEL_REV;
            double pollenMeasuredRpm =
                    pollenRawVelocity * 60.0
                            / POLLEN_TICKS_PER_FLYWHEEL_REV;

            PIDFCoefficients nectarPIDF =
                    shooter.getPIDFCoefficients(
                            DcMotor.RunMode.RUN_USING_ENCODER
                    );
            PIDFCoefficients pollenPIDF =
                    pollenShooter.getPIDFCoefficients(
                            DcMotor.RunMode.RUN_USING_ENCODER
                    );

            telemetry.addLine("========== NECTAR ==========");
            telemetry.addData("Enabled", nectarEnabled);
            telemetry.addData("Target RPM", "%.0f",
                    nectarEnabled ? WALL_SHOT : 0.0);
            telemetry.addData("Measured RPM", "%.0f", nectarMeasuredRpm);
            telemetry.addData("Motor Power", "%.3f", shooter.getPower());
            telemetry.addData("Nectar PIDF",
                    "%.6f / %.6f / %.6f / %.6f",
                    nectarPIDF.p, nectarPIDF.i,
                    nectarPIDF.d, nectarPIDF.f);

            telemetry.addLine("========== POLLEN ==========");
            telemetry.addData("Enabled", pollenEnabled);
            telemetry.addData("Target RPM", "%.0f",
                    pollenEnabled ? POLLEN_TARGET_RPM : 0.0);
            telemetry.addData("Measured RPM", "%.0f", pollenMeasuredRpm);
            telemetry.addData("Motor Power", "%.3f",
                    pollenShooter.getPower());
            telemetry.addData("Pollen PIDF",
                    "%.6f / %.6f / %.6f / %.6f",
                    pollenPIDF.p, pollenPIDF.i,
                    pollenPIDF.d, pollenPIDF.f);

            telemetry.addLine("========== OTHER ==========");
            telemetry.addData("Pollen Gate",
                    pollenGateRight ? "Right" : "Center");
            telemetry.addData("Gate position", "%.2f",
                    pollenGateRight
                            ? POLLEN_GATE_RIGHT
                            : POLLEN_GATE_CENTER);
            telemetry.addData("Kicker",
                    kickerEnabled ? "Running" : "Stopped");
            telemetry.addData("Intake",
                    gamepad1.right_trigger > 0.1 ? "Forward"
                            : gamepad1.left_trigger > 0.1
                              ? "Reverse" : "Stopped");
            telemetry.addData("Battery Voltage", "%.2f V",
                    battery.getVoltage());

            telemetry.update();
            idle();
        }

        // Stop everything
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