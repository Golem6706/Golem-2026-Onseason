package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.ShooterContants.SHOOTERHARDWARE_CONSTANTS;
import static frc.robot.subsystems.shooter.ShooterContants.SHOOTER_MOTORS_CURRENT_LIMIT;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOReal implements ShooterIO {
    private final TalonFX shooterMotor1;
    private final StatusSignal<Current> shooterMotor1Current;
    private final StatusSignal<Voltage> shooterMotor1OutputVoltage;

    private final TalonFX shooterMotor2;
    private final StatusSignal<Current> shooterMotor2Current;
    private final StatusSignal<Voltage> shooterMotor2OutputVoltage;

    private final TalonFX shooterMotor3;
    private final StatusSignal<Current> shooterMotor3Current;
    private final StatusSignal<Voltage> shooterMotor3OutputVoltage;

    private final TalonFX feederMotor1;
    private final StatusSignal<Current> feederMotor1Current;
    private final StatusSignal<Voltage> feederMotor1OutputVoltage;

    private final TalonFX feederMotor2;
    private final StatusSignal<Current> feederMotor2Current;
    private final StatusSignal<Voltage> feederMotor2OutputVoltage;

    public ShooterIOReal() {
        double freq = 100;

        // shooterMotor1
        this.shooterMotor1 = new TalonFX(SHOOTERHARDWARE_CONSTANTS.shooterMotor1ID());
        CurrentLimitsConfigs shooterMotor1CurrentLimit = new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SHOOTER_MOTORS_CURRENT_LIMIT);
        this.shooterMotor1.getConfigurator().apply(shooterMotor1CurrentLimit);
        this.shooterMotor1
                .getConfigurator()
                .apply(new MotorOutputConfigs()
                        .withInverted(
                                SHOOTERHARDWARE_CONSTANTS.shooterMotor1Inverted()
                                        ? InvertedValue.Clockwise_Positive
                                        : InvertedValue.CounterClockwise_Positive));
        this.shooterMotor1.setNeutralMode(NeutralModeValue.Brake);
        this.shooterMotor1Current = this.shooterMotor1.getSupplyCurrent();
        this.shooterMotor1OutputVoltage = this.shooterMotor1.getMotorVoltage();
        BaseStatusSignal.setUpdateFrequencyForAll(freq, shooterMotor1Current, shooterMotor1OutputVoltage);
        this.shooterMotor1.optimizeBusUtilization();
        // shooterMotor2
        this.shooterMotor2 = new TalonFX(SHOOTERHARDWARE_CONSTANTS.shooterMotor2ID());
        CurrentLimitsConfigs shooterMotor2CurrentLimit = new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SHOOTER_MOTORS_CURRENT_LIMIT);
        this.shooterMotor2.getConfigurator().apply(shooterMotor2CurrentLimit);
        this.shooterMotor2
                .getConfigurator()
                .apply(new MotorOutputConfigs()
                        .withInverted(
                                SHOOTERHARDWARE_CONSTANTS.shooterMotor2Inverted()
                                        ? InvertedValue.Clockwise_Positive
                                        : InvertedValue.CounterClockwise_Positive));
        this.shooterMotor2.setNeutralMode(NeutralModeValue.Brake);
        this.shooterMotor2Current = this.shooterMotor2.getSupplyCurrent();
        this.shooterMotor2OutputVoltage = this.shooterMotor2.getMotorVoltage();
        BaseStatusSignal.setUpdateFrequencyForAll(freq, shooterMotor2Current, shooterMotor2OutputVoltage);
        this.shooterMotor1.optimizeBusUtilization();
        // shooterMotor3
        this.shooterMotor3 = new TalonFX(SHOOTERHARDWARE_CONSTANTS.shooterMotor3ID());
        CurrentLimitsConfigs shooterMotor3CurrentLimit = new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SHOOTER_MOTORS_CURRENT_LIMIT);
        this.shooterMotor3.getConfigurator().apply(shooterMotor3CurrentLimit);
        this.shooterMotor3
                .getConfigurator()
                .apply(new MotorOutputConfigs()
                        .withInverted(
                                SHOOTERHARDWARE_CONSTANTS.shooterMotor3Inverted()
                                        ? InvertedValue.Clockwise_Positive
                                        : InvertedValue.CounterClockwise_Positive));
        this.shooterMotor3.setNeutralMode(NeutralModeValue.Brake);
        this.shooterMotor3Current = this.shooterMotor3.getSupplyCurrent();
        this.shooterMotor3OutputVoltage = this.shooterMotor3.getMotorVoltage();
        BaseStatusSignal.setUpdateFrequencyForAll(freq, shooterMotor3Current, shooterMotor3OutputVoltage);
        this.shooterMotor3.optimizeBusUtilization();

        // feederMotor1
        this.feederMotor1 = new TalonFX(SHOOTERHARDWARE_CONSTANTS.feederMotor1ID());
        CurrentLimitsConfigs feederMotor1CurrentLimit = new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SHOOTER_MOTORS_CURRENT_LIMIT);
        this.feederMotor1.getConfigurator().apply(feederMotor1CurrentLimit);
        this.feederMotor1
                .getConfigurator()
                .apply(new MotorOutputConfigs()
                        .withInverted(
                                SHOOTERHARDWARE_CONSTANTS.feederMotor1Inverted()
                                        ? InvertedValue.Clockwise_Positive
                                        : InvertedValue.CounterClockwise_Positive));
        this.feederMotor1.setNeutralMode(NeutralModeValue.Brake);
        this.feederMotor1Current = this.feederMotor1.getSupplyCurrent();
        this.feederMotor1OutputVoltage = this.feederMotor1.getMotorVoltage();
        BaseStatusSignal.setUpdateFrequencyForAll(freq, feederMotor1Current, feederMotor1OutputVoltage);
        this.feederMotor1.optimizeBusUtilization();
        // feederMotor2
        this.feederMotor2 = new TalonFX(SHOOTERHARDWARE_CONSTANTS.feederMotor1ID());
        CurrentLimitsConfigs feederMotor2CurrentLimit = new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SHOOTER_MOTORS_CURRENT_LIMIT);
        this.feederMotor2.getConfigurator().apply(feederMotor2CurrentLimit);
        this.feederMotor2
                .getConfigurator()
                .apply(new MotorOutputConfigs()
                        .withInverted(
                                SHOOTERHARDWARE_CONSTANTS.feederMotor2Inverted()
                                        ? InvertedValue.Clockwise_Positive
                                        : InvertedValue.CounterClockwise_Positive));
        this.feederMotor2.setNeutralMode(NeutralModeValue.Brake);
        this.feederMotor2Current = this.feederMotor2.getSupplyCurrent();
        this.feederMotor2OutputVoltage = this.feederMotor2.getMotorVoltage();
        BaseStatusSignal.setUpdateFrequencyForAll(freq, feederMotor2Current, feederMotor2OutputVoltage);
        this.feederMotor2.optimizeBusUtilization();
    }

    private double avarageFeederCurrent = 0.0, avarageFeederOutputVolts = 0.0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Implementation to read real hardware inputs
        inputs.shooterMotor1Connected = BaseStatusSignal.refreshAll(shooterMotor1Current, shooterMotor1OutputVoltage)
                .isOK();
        inputs.shooterMotor2Connected = BaseStatusSignal.refreshAll(shooterMotor2Current, shooterMotor2OutputVoltage)
                .isOK();
        inputs.shooterMotor3Connected = BaseStatusSignal.refreshAll(shooterMotor3Current, shooterMotor3OutputVoltage)
                .isOK();
        inputs.feederMotor1Connected = BaseStatusSignal.refreshAll(feederMotor1Current, feederMotor1OutputVoltage)
                .isOK();
        inputs.feederMotor2Connected = BaseStatusSignal.refreshAll(feederMotor2Current, feederMotor2OutputVoltage)
                .isOK();

        inputs.shooterMotor1CurrentAmps = shooterMotor1Current.getValueAsDouble();
        inputs.shooterMotor2CurrentAmps = shooterMotor2Current.getValueAsDouble();
        inputs.shooterMotor3CurrentAmps = shooterMotor3Current.getValueAsDouble();

        avarageFeederCurrent = feederMotor1Current.getValueAsDouble() + feederMotor2Current.getValueAsDouble();
        inputs.feederMotor1CurrentAmps = inputs.feederMotor2CurrentAmps = avarageFeederCurrent / 2;

        inputs.shooterMotor1OutputVolts = shooterMotor1OutputVoltage.getValueAsDouble();
        inputs.shooterMotor2OutputVolts = shooterMotor2OutputVoltage.getValueAsDouble();
        inputs.shooterMotor3OutputVolts = shooterMotor3OutputVoltage.getValueAsDouble();

        avarageFeederOutputVolts =
                feederMotor1OutputVoltage.getValueAsDouble() + feederMotor2OutputVoltage.getValueAsDouble();
        inputs.feederMotor1OutputVolts = inputs.feederMotor2OutputVolts = avarageFeederOutputVolts / 2;
    }

    private VoltageOut voltageOut = new VoltageOut(Volts.zero());

    @Override
    public void setShooterMotorsVoltage(double volts) {
        // Implementation to set voltage to real hardware shooter motors
        voltageOut.withOutput(volts - 0.5);
        shooterMotor1.setControl(voltageOut);
        voltageOut.withOutput(volts);
        shooterMotor2.setControl(voltageOut);
        voltageOut.withOutput(volts + 0.5);
        shooterMotor3.setControl(voltageOut);
    }

    @Override
    public void setFeederMotorsVoltage(double volts) {
        // Implementation to set voltage to real hardware feeder motors
        voltageOut.withOutput(volts);
        feederMotor1.setControl(voltageOut);
        feederMotor2.setControl(voltageOut);
    }
}
