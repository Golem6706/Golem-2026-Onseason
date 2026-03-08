package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterContants.*;

import edu.wpi.first.math.MathUtil;

public class ShooterIOSim implements ShooterIO {
    private double shooterVelocityRPM = 0.0;
    private double shooterAppliedVolts = 0.0;

    private double feederVelocityRPM = 0.0;
    private double feederAppliedVolts = 0.0;

    private static final double MOTOR_KV = 0.012;
    private static final double SIMULATION_DT = 0.02;

    public ShooterIOSim() {}

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.shootersConnected = new boolean[] {true};
        inputs.shooterMotorsVelocityRPM = new double[] {shooterVelocityRPM};

        double backEMF = shooterVelocityRPM * MOTOR_KV;
        double effectiveVoltage = shooterAppliedVolts - backEMF;
        double acceleration = effectiveVoltage / MOTOR_KV;

        shooterVelocityRPM += acceleration * SIMULATION_DT;
        shooterVelocityRPM = Math.max(0, shooterVelocityRPM);

        inputs.shooterMotorsAverageVolts = shooterAppliedVolts;
        inputs.shooterMotorsTotalCurrentAmps = Math.abs(effectiveVoltage) / 0.05;

        inputs.feedersConnected = new boolean[] {true};
        inputs.feederMotorsVelocityRPM = new double[] {feederVelocityRPM};

        double feederBackEMF = feederVelocityRPM * MOTOR_KV;
        double feederEffectiveVoltage = feederAppliedVolts - feederBackEMF;
        double feederAcceleration = feederEffectiveVoltage / MOTOR_KV;

        feederVelocityRPM += feederAcceleration * SIMULATION_DT;
        feederVelocityRPM = Math.max(0, feederVelocityRPM);

        inputs.feederMotorsAverageVolts = feederAppliedVolts;
        inputs.feederMotorsTotalCurrentAmps = Math.abs(feederEffectiveVoltage) / 0.05;
    }

    @Override
    public void setShooterMotorsVoltage(double volts) {
        shooterAppliedVolts = volts;
    }

    @Override
    public void setFeederMotorsVoltage(double volts) {
        feederAppliedVolts = volts;
    }

    @Override
    public void setShooterVelocity(double rpm) {
        double error = rpm - shooterVelocityRPM;
        double controlVolts = error * 0.1;
        shooterAppliedVolts = MathUtil.clamp(controlVolts, -12.0, 12.0);
    }
}
