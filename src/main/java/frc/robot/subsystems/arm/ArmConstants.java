package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import java.util.OptionalInt;

public final class ArmConstants {
    // General Constants (shared across all robots)
    public static final Distance HEIGHT_THRESHOLD_ENABLE_LOW_SPEED_MODE = Centimeters.of(45);
    public static final LinearVelocity Arm_MOVING_VELOCITY_THRESHOLD = MetersPerSecond.of(0.03);

    public static final double ALGAE_MODE_SPEED_FACTOR = 0.5;

    // Current Limits (shared across all robots)
    public static final Current STATOR_CURRENT_LIMIT = Amps.of(80);
    public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(60);
    public static final Time OVERHEAT_PROTECTION_TIME = Seconds.of(1);
    public static final Current OVERHEAT_PROTECTION_CURRENT = Amps.of(40);

    public static final Current ARM_CURRENT_LIMIT = Amps.of(20.0);
    public static final Voltage ARM_MAX_VOLTAGE = Volts.of(4.0);

    public static final Current INTAKE_CURRENT_LIMIT = Amps.of(45);
    public static final Voltage INTAKE_MAX_VOLTAGE = Volts.of(10.0);
    public static final Voltage INTAKE_VOLTAGE = Volts.of(9.5);
    public static final Voltage REVERT_INTAKE_VOLTAGE = Volts.of(-8.5);

    public static final Voltage INTAKE_VOLTAGE_FOR_AUTO_ONLY = Volts.of(10.5);
    // The setpoint angle for arm to intake from ground
    public static final Angle ARM_INTAKING_ANGLE = Degrees.of(13);
    public static final Angle ARM_ANGLE_HOLDING = Degrees.of(35);

    // Toggle angle when shooting, this helps feeder
    public static final Angle ARM_TOGGLE_ANGLE_01 = Degrees.of(45);
    public static final Angle ARM_TOGGLE_ANGLE_02 = Degrees.of(55);
    public static final Angle ARM_TOGGLE_ANGLE_03 = Degrees.of(65);
    public static final Angle ARM_TOGGLE_ANGLE_04 = Degrees.of(75);
    public static final Angle ARM_TOGGLE_ANGLE_05 = Degrees.of(85);

    public static final Angle ARM_STARTING_ANGLE = Degrees.of(90);

    public record ArmHardwareConstants(
            Distance ARM_COM_LENGTH,
            Mass ARM_MASS,
            DCMotor ARM_GEARBOX,
            double ARM_GEARING_REDUCTION,
            Angle ARM_UPPER_HARD_LIMIT,
            Angle ARM_LOWER_HARD_LIMIT,
            Angle ABSOLUTE_ENCODER_READING_AT_UPPER_LIM,
            //     Angle ABSOLUTE_ENCODER_READING_AT_LOWER_LIM,
            //     int ABSOLUTE_ENCODER_CHANNEL,
            int ABSOLUTE_ENCODER_ID,
            boolean ABSOLUTE_ENCODER_INVERTED,
            OptionalInt ARM_MOTOR_LEFT_ID,
            boolean ARM_MOTOR_LEFT_INVERTED,
            int ARM_MOTOR_RIGHT_ID,
            boolean ARM_MOTOR_RIGHT_INVERTED,
            int INTAKE_MOTOR_ID,
            boolean INTAKE_MOTOR_INVERTED) {}

    public static final ArmHardwareConstants HARDWARE_CONSTANTS = new ArmHardwareConstants(
            Centimeters.of(33),
            Kilograms.of(3.0),
            DCMotor.getKrakenX60(2),
            9 * 32 / 16,
            // Following data need to be measured on real Robot
            Degrees.of(90.0),
            Degrees.of(0.0),
            Rotation.of(0.042), // 0.054
            //     Rotation.of(2.9), // -3.1
            21,
            false,
            OptionalInt.of(22),
            true,
            19,
            false,
            20,
            true);

    public record ArmPIDConstants(
            double kS,
            double kG,
            double kV,
            double kA,
            double kP,
            AngularVelocity VELOCTIY_CONSTRAIN,
            AngularAcceleration ACCELERATION_CONSTRAIN,
            Angle TOLERANCE) {}

    public static final ArmPIDConstants PID_CONSTANTS = new ArmPIDConstants(
            0.05,
            0.08,
            0.34,
            0.01,
            6.0 / Math.toRadians(30),
            RotationsPerSecond.of(0.5),
            RotationsPerSecondPerSecond.of(3),
            Degrees.of(3));
}
