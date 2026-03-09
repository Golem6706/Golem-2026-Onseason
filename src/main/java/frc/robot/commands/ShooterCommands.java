package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterContants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterCommands {
    private static final Translation2d BLUE_TARGET_POSITION = new Translation2d(17.548 - 11.915, 4.0);

    private static final InterpolatingDoubleTreeMap distanceToRPMTable = new InterpolatingDoubleTreeMap();

    static {
        distanceToRPMTable.put(2.5, 2500.0);
        distanceToRPMTable.put(3.0, 3000.0);
        distanceToRPMTable.put(3.5, 3500.0);
        distanceToRPMTable.put(4.0, 4000.0);
        distanceToRPMTable.put(4.5, 4500.0);
    }

    private ShooterCommands() {}

    public static double getTargetRPM(Drive drive) {
        Translation2d robotPosition = drive.getPose().getTranslation();
        Translation2d targetPosition = BLUE_TARGET_POSITION;
        double distance = robotPosition.getDistance(targetPosition);

        double rpm = distanceToRPMTable.get(distance);
        Logger.recordOutput("Shooter/TargetDistance", distance);
        Logger.recordOutput("Shooter/InterpolatedRPM", rpm);

        return rpm;
    }

    private static boolean isAtReference(Shooter shooter) {
        return Math.abs(shooter.getShooterVelocityRPM() - shooter.getShooterTargetRPM())
                <= ShooterContants.PREPARE_TO_SHOOT_TOLERANCE_RPM;
    }

    public static Command prepareToShoot(Shooter shooter, Drive drive) {
        return shooter.run(() -> shooter.setShooterTargetRPM(getTargetRPM(drive)))
                .until(() -> isAtReference(shooter));
    }

    public static Command shootWhenReady(Shooter shooter, Drive drive, BooleanSupplier trigger) {
        return shooter.run(() -> {
            shooter.setShooterTargetRPM(getTargetRPM(drive));
            if (trigger.getAsBoolean() && isAtReference(shooter)) {
                shooter.setFeederVoltage(ShooterContants.VOLTAGE_SETTINGS.feederMotorVolts()[0]);
            } else {
                shooter.setFeederVoltage(0.0);
            }
        });
    }
}
