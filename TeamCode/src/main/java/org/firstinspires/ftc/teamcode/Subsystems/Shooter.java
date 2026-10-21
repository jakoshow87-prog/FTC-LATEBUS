package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
// not complete
public class Shooter {
    private static final String MOTOR_NAME = "shooter";

    // Assumes 6000 RPM is the bare motor speed before the 5:1 cartridge.
    private static final double MOTOR_FREE_RPM = 6000.0;
    private static final double INTERNAL_REDUCTION = 5.0;
    private static final double EXTERNAL_REDUCTION = 4.0;

    public static final double MAX_FLYWHEEL_RPM =
            MOTOR_FREE_RPM / INTERNAL_REDUCTION / EXTERNAL_REDUCTION;

    // REV HD Hex example: 28 encoder ticks per bare motor revolution.
    // Replace this with the value for your motor and encoder.
    private static final double ENCODER_TICKS_PER_BARE_MOTOR_REV = 28.0;

    private static final double ENCODER_TICKS_PER_FLYWHEEL_REV =
            ENCODER_TICKS_PER_BARE_MOTOR_REV
                    * INTERNAL_REDUCTION
                    * EXTERNAL_REDUCTION;

    private static final double MAX_ENCODER_TICKS_PER_SECOND =
            MOTOR_FREE_RPM / 60.0 * ENCODER_TICKS_PER_BARE_MOTOR_REV;

    // Initial estimate; tune this on your robot.
    private static final double PID_F =
            32767.0 / MAX_ENCODER_TICKS_PER_SECOND;

    // Start by using the Hub's existing PIDF settings.
    // Set this true only after entering your tuned P, I, D, and F values.
    private static final boolean APPLY_CUSTOM_PIDF = false;
    private static final double PID_P = 0.0;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;

    private static final double RPM_STEP_PER_SECOND = 120.0;
    private static final double RPM_TOLERANCE = 20.0;

    private final DcMotorEx motor;
    private double targetRpm = 0.0;

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);

        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (APPLY_CUSTOM_PIDF) {
            motor.setVelocityPIDFCoefficients(PID_P, PID_I, PID_D, PID_F);
        }
    }

    public void adjustTargetRpm(double rpmChange) {
        setTargetRpm(targetRpm + rpmChange);
    }

    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0.0, Math.min(rpm, MAX_FLYWHEEL_RPM));

        double targetTicksPerSecond =
                targetRpm * ENCODER_TICKS_PER_FLYWHEEL_REV / 60.0;

        motor.setVelocity(targetTicksPerSecond);
    }

    public void stop() {
        targetRpm = 0.0;
        motor.setVelocity(0.0);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getMeasuredRpm() {
        return motor.getVelocity() * 60.0 / ENCODER_TICKS_PER_FLYWHEEL_REV;
    }

    public double getErrorRpm() {
        return targetRpm - getMeasuredRpm();
    }

    public boolean isAtSpeed() {
        return targetRpm > 0.0
                && Math.abs(getErrorRpm()) <= RPM_TOLERANCE;
    }

    public double getF() {
        return APPLY_CUSTOM_PIDF
                ? PID_F
                : motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f;
    }

    public double getP() {
        return APPLY_CUSTOM_PIDF
                ? PID_P
                : motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p;
    }

    public double getI() {
        return APPLY_CUSTOM_PIDF
                ? PID_I
                : motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i;
    }

    public double getD() {
        return APPLY_CUSTOM_PIDF
                ? PID_D
                : motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d;
    }

    public static double getRpmStepPerSecond() {
        return RPM_STEP_PER_SECOND;
    }
}