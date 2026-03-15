package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
// import org.ironmaple.utils.FieldMirroringUtils;
import frc.robot.util.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public class ShooterCommands {
    private static final Translation2d BLUE_TARGET_POSITION = new Translation2d(4.625, 4.03); // (4.6, 4.0)

    private static final InterpolatingDoubleTreeMap distanceToRPMTable = new InterpolatingDoubleTreeMap();

    static {
        distanceToRPMTable.put(1.5, 2500.0);
        distanceToRPMTable.put(2.0, 2600.0); //
        distanceToRPMTable.put(2.5, 2700.0);
        distanceToRPMTable.put(3.0, 2900.0);
        distanceToRPMTable.put(3.5, 3100.0);
        distanceToRPMTable.put(4.0, 3300.0);
        // distanceToRPMTable.put(4.5, 3500.0);
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
        return shooter.runSetPoint(() -> getTargetRPM(drive), () -> trigger.getAsBoolean() ? 8.5 : -0.5);
    }

    // public static Command shootToDriverStationCommand(Shooter shooter, Drive drive) {
    //     return shooter.runSetPoint(() -> getTargetRPM(drive), () -> 7.0);
    // }
}
