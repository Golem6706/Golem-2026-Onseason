package frc.robot.subsystems.arm;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.arm.ArmConstants.*;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmIOReal implements ArmIO {
    /**
     * The difference between the raw encoder reading angle and the actual angle.
     * Real Angle = Encoder Angle - Offset Angle
     * Offset Angle = Encoder Angle - Real Angle
     */
    private static final Rotation2d ABSOLUTE_ENCODER_OFFSET = new Rotation2d(HARDWARE_CONSTANTS
                    .ABSOLUTE_ENCODER_READING_AT_UPPER_LIM()
                    .times(HARDWARE_CONSTANTS.ABSOLUTE_ENCODER_INVERTED() ? -1 : 1))
                .minus(new Rotation2d(HARDWARE_CONSTANTS.ARM_UPPER_HARD_LIMIT()));

    //Arm Hardware
    private final TalonFX armTalon;
    private final TalonFX intakeTalon;
    private final DutyCycleEncoder absoluteEncoder;

    //CTRE Motor Signals
    private final StatusSignal<Angle> relativeEncoderAngle;
    private final StatusSignal<AngularVelocity> relativeEncoderVelocity;
    private final StatusSignal<Current> armMotorSupplyCurrent;
    private final StatusSignal<Voltage> armMotorSupplyVoltage;

    private final StatusSignal<Current> intakeMotorSupplyCurrent;
    private final StatusSignal<Voltage> intakeMotorSupplyVoltage;

    public ArmIOReal() {
        //Initialize Hardwares
        this.armTalon = new TalonFX(HARDWARE_CONSTANTS.ARM_MOTOR_ID());
        this.intakeTalon = new TalonFX(HARDWARE_CONSTANTS.INTAKE_MOTOR_ID());
        this.absoluteEncoder = new DutyCycleEncoder(HARDWARE_CONSTANTS.ABSOLUTE_ENCODER_CHANNEL());
        //Configure Motor
        armTalon.getConfigurator().apply(new MotorOutputConfigs()
                                            .withInverted(HARDWARE_CONSTANTS.ARM_MOTOR_INVERTED()
                                            ? InvertedValue.Clockwise_Positive
                                            : InvertedValue.CounterClockwise_Positive));
        armTalon.getConfigurator().apply(new CurrentLimitsConfigs()
                                            .withSupplyCurrentLimitEnable(true)
                                            .withSupplyCurrentLimit(ARM_CURRENT_LIMIT));
        intakeTalon.getConfigurator().apply(new MotorOutputConfigs()
                                            .withInverted(HARDWARE_CONSTANTS.INTAKE_MOTOR_INVERTED()
                                            ? InvertedValue.Clockwise_Positive
                                            : InvertedValue.CounterClockwise_Positive));
        intakeTalon.getConfigurator().apply(new CurrentLimitsConfigs()
                                            .withSupplyCurrentLimitEnable(true)
                                            .withSupplyCurrentLimit(INTAKE_CURRENT_LIMIT));

        //Obtain Motor Status Signals
        this.relativeEncoderAngle = armTalon.getPosition();
        this.relativeEncoderVelocity = armTalon.getVelocity();
        this.armMotorSupplyCurrent = armTalon.getSupplyCurrent();
        this.armMotorSupplyVoltage = armTalon.getSupplyVoltage();
        BaseStatusSignal.setUpdateFrequencyForAll(
            100, relativeEncoderAngle, relativeEncoderVelocity,armMotorSupplyCurrent,armMotorSupplyVoltage);
        armTalon.optimizeBusUtilization();

        this.intakeMotorSupplyCurrent = intakeTalon.getSupplyCurrent();
        this.intakeMotorSupplyVoltage = intakeTalon.getSupplyVoltage();
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,intakeMotorSupplyCurrent,intakeMotorSupplyVoltage);
        intakeTalon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        //Get absolut Encoder readings
        inputs.absoluteEncoderAngle = absoluteEncoder.isConnected() 
                ? Optional.of(Rotation2d.fromRotations(
                        absoluteEncoder.get() * (HARDWARE_CONSTANTS.ABSOLUTE_ENCODER_INVERTED() ? -1 : 1))
                    .minus(ABSOLUTE_ENCODER_OFFSET))
                : Optional.empty(); 
        //Refresh armMotor signals
        StatusCode statusCode = BaseStatusSignal.refreshAll(
                relativeEncoderAngle, relativeEncoderVelocity, armMotorSupplyCurrent, armMotorSupplyVoltage);
        //Obtain Motor Readings
        inputs.armMotorConnected = statusCode.isOK();
        inputs.relativeEncoderAngledRad = Units.rotationsToRadians(relativeEncoderAngle.getValueAsDouble());
        inputs.relativeEncoderVelocityRadPerSec = Units.rotationsToRadians(relativeEncoderVelocity.getValueAsDouble());
        inputs.armMotorSupplyCurrentAmps = armMotorSupplyCurrent.getValueAsDouble();
        inputs.armMotorOutputVolts = armMotorSupplyVoltage.getValueAsDouble(); 

        //Refresh intakeMotor signals
        statusCode = BaseStatusSignal.refreshAll(
                intakeMotorSupplyCurrent, intakeMotorSupplyVoltage);
        //Obtain Motor Readings
        inputs.intakeMotorConnected = statusCode.isOK();
        inputs.intakeMotorSupplyCurrentAmps = intakeMotorSupplyCurrent.getValueAsDouble();
        inputs.intakeMotorOutputVolts = intakeMotorSupplyVoltage.getValueAsDouble(); 

        SmartDashboard.putNumber("Arm/Raw Encoder Reading", absoluteEncoder.get());
        SmartDashboard.putBoolean("Arm/Absolute Encoder Connected", absoluteEncoder.isConnected());
    }

    private VoltageOut voltageOut = new VoltageOut(Volts.zero());

    @Override
    public void setArmMotorOutput (Voltage voltage) {
        voltageOut.withOutput(voltage);
        armTalon.setControl(voltageOut);
    }

    @Override
    public void setArmMotorBrake (boolean brakeModeEnable) {
        NeutralModeValue value = brakeModeEnable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        armTalon.setNeutralMode(value, 0.1);
    }

    @Override
    public void setIntakeMotorOutput (Voltage voltage) {
        voltageOut.withOutput(voltage);
        intakeTalon.setControl(voltageOut);
    }

    @Override
    public void setIntakeMotorBrake (boolean brakeModeEnable) {
        NeutralModeValue value = brakeModeEnable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        intakeTalon.setNeutralMode(value, 0.1);
    }
}
