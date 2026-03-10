package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Current;

public final class ShooterContants {
    public record ShooterHardwareConstants(
            int[] shooterMotorIDs,
            boolean[] shooterMotorInverted,
            int[] feederMotorIDs,
            boolean[] feederMotorInverted) {

        public ShooterHardwareConstants {
            if (shooterMotorIDs.length != shooterMotorInverted.length) {
                throw new IllegalArgumentException("shooterMotorIDs and shooterMotorInverted must have same length");
            }
            if (feederMotorIDs.length != feederMotorInverted.length) {
                throw new IllegalArgumentException("feederMotorIDs and feederMotorInverted must have same length");
            }
        }
    }

    public static final ShooterHardwareConstants SHOOTERHARDWARE_CONSTANTS = new ShooterHardwareConstants(
            new int[] {15, 16}, new boolean[] {false, false}, new int[] {14, 13}, new boolean[] {false, false});

    public static final Slot0Configs SHOOTER_VELOCITY_GAINS =
            new Slot0Configs().withKP(0.25).withKI(0.0).withKD(0.0).withKS(0.2).withKV(0.124);

    public static final double MAX_SHOOTER_RPM = 6000.0;
    public static final double MAX_FEEDER_RPM = 3000.0;

    public static final double SHOOTER_ACCELERATION_RPM_PER_SEC = 5000.0;

    public static final double SHOOTER_VELOCITY_TOLERANCE_RPM = 100.0;

    public static final double PREPARE_TO_SHOOT_TOLERANCE_RPM = 500.0;

    public static final Current SHOOTER_MOTORS_CURRENT_LIMIT = Amps.of(40);
    public static final Current FEEDER_MOTORS_CURRENT_LIMIT = Amps.of(15);
}
