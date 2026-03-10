package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public class ShooterCommands {
    private static final Translation2d BLUE_TARGET_POSITION = new Translation2d(4.6, 4.0);

    private static final InterpolatingDoubleTreeMap distanceToRPMTable = new InterpolatingDoubleTreeMap();

    static {
        distanceToRPMTable.put(2.5, 2600.0);
        distanceToRPMTable.put(3.0, 2800.0);
        distanceToRPMTable.put(3.5, 3000.0);
        distanceToRPMTable.put(4.0, 3200.0);
        distanceToRPMTable.put(4.5, 3400.0);
    }

    private ShooterCommands() {}

    public static double getTargetRPM(Drive drive) {
        Translation2d robotPosition = drive.getPose().getTranslation();
        Translation2d targetPosition = FieldMirroringUtils.toCurrentAllianceTranslation(BLUE_TARGET_POSITION);
        double distance = robotPosition.getDistance(targetPosition);

        double rpm = distanceToRPMTable.get(distance);
        Logger.recordOutput("Shooter/TargetDistance", distance);
        Logger.recordOutput("Shooter/InterpolatedRPM", rpm);

        return rpm;
    }

    public static Command shootCommand(Shooter shooter, Drive drive, Trigger trigger) {
        return shooter.runSetPoint(() -> getTargetRPM(drive), () -> trigger.getAsBoolean() ? 6.0 : -0.5);
    }
}
