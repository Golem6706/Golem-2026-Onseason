package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.Current;

public final class ShooterContants {
    // Array-based hardware configuration
    public record ShooterHardwareConstants(
            int[] shooterMotorIDs,
            boolean[] shooterMotorInverted,
            int[] feederMotorIDs,
            boolean[] feederMotorInverted) {

        public ShooterHardwareConstants {
            // Validate array lengths match
            if (shooterMotorIDs.length != shooterMotorInverted.length) {
                throw new IllegalArgumentException("shooterMotorIDs and shooterMotorInverted must have same length");
            }
            if (feederMotorIDs.length != feederMotorInverted.length) {
                throw new IllegalArgumentException("feederMotorIDs and feederMotorInverted must have same length");
            }
        }
    }

    // Default configuration with 3 shooter motors and 2 feeder motors
    public static final ShooterHardwareConstants SHOOTERHARDWARE_CONSTANTS = new ShooterHardwareConstants(
            new int[] {14, 15, 16}, new boolean[] {false, false, false}, new int[] {18, 19}, new boolean[] {false, false
            });

    // PID constants for velocity control (RPM)
    public static final Slot0Configs SHOOTER_VELOCITY_GAINS = new Slot0Configs()
            .withKP(0.1) // Proportional gain
            .withKI(0.0) // Integral gain
            .withKD(0.0) // Derivative gain
            .withKS(0.0) // Static feedforward
            .withKV(0.12); // Velocity feedforward (volts per RPM)

    public static final Slot0Configs FEEDER_VELOCITY_GAINS =
            new Slot0Configs().withKP(0.1).withKI(0.0).withKD(0.0).withKS(0.0).withKV(0.12);

    // Maximum RPM for shooter and feeder
    public static final double MAX_SHOOTER_RPM = 6000.0;
    public static final double MAX_FEEDER_RPM = 3000.0;

    // Voltage settings for backward compatibility
    public record VoltageSettings(double[] shooterMotorVolts, double[] feederMotorVolts) {

        public VoltageSettings {
            if (shooterMotorVolts == null || feederMotorVolts == null) {
                throw new IllegalArgumentException("Voltage arrays cannot be null");
            }
        }
    }

    // Default voltage settings matching previous configuration
    public static final VoltageSettings VOLTAGE_SETTINGS =
            new VoltageSettings(new double[] {10.0, 10.0, 10.0}, new double[] {4.0, 6.0});

    public static final Current SHOOTER_MOTORS_CURRENT_LIMIT = Amps.of(20);
    public static final Current FEEDER_MOTORS_CURRENT_LIMIT = Amps.of(10);
}
