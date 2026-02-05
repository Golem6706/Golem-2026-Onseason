package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;

public final class ShooterContants {
    public record ShooterHardwareConstants(
            int shooterMotor1ID,
            boolean shooterMotor1Inverted,
            int shooterMotor2ID,
            boolean shooterMotor2Inverted,
            int shooterMotor3ID,
            boolean shooterMotor3Inverted,
            int feederMotor1ID,
            boolean feederMotor1Inverted,
            int feederMotor2ID,
            boolean feederMotor2Inverted) {}

    public static final ShooterHardwareConstants SHOOTERHARDWARE_CONSTANTS =
            new ShooterHardwareConstants(14, false, 15, false, 16, false, 18, false, 19, false);

    public record VoltageSettings(
            double SHOOTERMOTOR1_VOLTS,
            double SHOOTERMOTOR2_VOLTS,
            double SHOOTERMOTOR3_VOLTS,
            double FEEDERMOTOR1_VOLTS,
            double FEEDERMOTOR2_VOLTS) {};
    public static final VoltageSettings VOLTAGE_SETTINGS = new VoltageSettings(10, 10.0, 10.0, 4.0, 6.0);

    public static final Current SHOOTER_MOTORS_CURRENT_LIMIT = Amps.of(20);
    public static final Current FEEDER_MOTORS_CURRENT_LIMIT = Amps.of(10);
}
