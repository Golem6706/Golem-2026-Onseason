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
            new int[] {15, 16, 17, 18}, new boolean[] {false, false, false, false}, new int[] {14, 13}, new boolean[] {
                false, false
            });

    public static final VoltageSettings VOLTAGE_SETTINGS =
            new VoltageSettings(new double[] {10.0, 10.0, 10.0, 10.0}, new double[] {6.0, 6.0});

    public static final Slot0Configs SHOOTER_VELOCITY_GAINS =
            new Slot0Configs().withKP(0.1).withKI(0.0).withKD(0.0).withKS(0.0).withKV(0.12);

    public static final Slot0Configs FEEDER_VELOCITY_GAINS =
            new Slot0Configs().withKP(0.1).withKI(0.0).withKD(0.0).withKS(0.0).withKV(0.12);

    public static final double MAX_SHOOTER_RPM = 6000.0;
    public static final double MAX_FEEDER_RPM = 3000.0;

    public static final double SHOOTER_ACCELERATION_RPM_PER_SEC = 12000.0;

    public static final double SHOOTER_VELOCITY_TOLERANCE_RPM = 100.0;

    public static final double PREPARE_TO_SHOOT_TOLERANCE_RPM = 500.0;

    public record VoltageSettings(double[] shooterMotorVolts, double[] feederMotorVolts) {

        public VoltageSettings {
            if (shooterMotorVolts == null || feederMotorVolts == null) {
                throw new IllegalArgumentException("Voltage arrays cannot be null");
            }
        }
    }

    public static final Current SHOOTER_MOTORS_CURRENT_LIMIT = Amps.of(20);
    public static final Current FEEDER_MOTORS_CURRENT_LIMIT = Amps.of(15);
}
