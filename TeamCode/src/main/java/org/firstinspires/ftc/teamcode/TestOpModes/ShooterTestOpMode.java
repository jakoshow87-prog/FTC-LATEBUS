package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
//not complete
@TeleOp(name = "Shooter PIDF Calibrator", group = "Test")
public class ShooterTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        Shooter shooter = new Shooter(hardwareMap);

        telemetry.addLine("D-pad up: increase target RPM");
        telemetry.addLine("D-pad down: decrease target RPM");
        telemetry.addLine("A: stop shooter");
        telemetry.update();

        waitForStart();

        long previousTime = System.nanoTime();

        while (opModeIsActive()) {
            long now = System.nanoTime();
            double dt = (now - previousTime) / 1_000_000_000.0;
            previousTime = now;

            double adjustment = 0.0;

            if (gamepad1.dpad_up) {
                adjustment += 1.0;
            }
            if (gamepad1.dpad_down) {
                adjustment -= 1.0;
            }

            shooter.adjustTargetRpm(
                    adjustment * Shooter.getRpmStepPerSecond() * dt
            );

            if (gamepad1.a) {
                shooter.stop();
            }

            telemetry.addData("Target RPM", "%.0f", shooter.getTargetRpm());
            telemetry.addData("Measured RPM", "%.0f", shooter.getMeasuredRpm());
            telemetry.addData("RPM error", "%.0f", shooter.getErrorRpm());
            telemetry.addData("At speed", shooter.isAtSpeed());
            telemetry.addData(
                    "PIDF (P, I, D, F)",
                    "%.3f, %.3f, %.3f, %.3f",
                    shooter.getP(),
                    shooter.getI(),
                    shooter.getD(),
                    shooter.getF()
            );
            telemetry.addLine("D-pad up/down: adjust RPM | A: stop");
            telemetry.update();
        }

        shooter.stop();
    }
}