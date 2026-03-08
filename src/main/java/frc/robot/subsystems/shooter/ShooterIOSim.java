package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterContants.*;

import edu.wpi.first.math.MathUtil;

public class ShooterIOSim implements ShooterIO {
    // Simple simulation state for shooter motor
    private double shooterVelocityRPM = 0.0;
    private double shooterAppliedVolts = 0.0;

    // Simple simulation state for feeder motor
    private double feederVelocityRPM = 0.0;
    private double feederAppliedVolts = 0.0;

    // Simulation parameters
    private static final double MOTOR_KV = 0.012; // Volts per RPM
    private static final double SIMULATION_DT = 0.02; // 20ms update period

    public ShooterIOSim() {
        // No additional initialization needed
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Update shooter inputs - use single values since simulation is idealized
        inputs.shootersConnected = new boolean[] {true};
        inputs.shooterMotorsVelocityRPM = new double[] {shooterVelocityRPM};

        // Simple motor model: velocity = voltage / KV
        double backEMF = shooterVelocityRPM * MOTOR_KV;
        double effectiveVoltage = shooterAppliedVolts - backEMF;
        double acceleration = effectiveVoltage / MOTOR_KV;

        // Update velocity
        shooterVelocityRPM += acceleration * SIMULATION_DT;
        shooterVelocityRPM = Math.max(0, shooterVelocityRPM);

        inputs.shooterMotorsAverageVolts = shooterAppliedVolts;
        inputs.shooterMotorsTotalCurrentAmps = Math.abs(effectiveVoltage) / 0.05; // 0.05 Ohm resistance

        // Update feeder inputs
        inputs.feedersConnected = new boolean[] {true};
        inputs.feederMotorsVelocityRPM = new double[] {feederVelocityRPM};

        // Simple motor model for feeder
        double feederBackEMF = feederVelocityRPM * MOTOR_KV;
        double feederEffectiveVoltage = feederAppliedVolts - feederBackEMF;
        double feederAcceleration = feederEffectiveVoltage / MOTOR_KV;

        // Update velocity
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
        // For velocity control, use a simple P controller to simulate closed-loop
        double error = rpm - shooterVelocityRPM;
        double controlVolts = error * 0.1; // Simple P gain
        shooterAppliedVolts = MathUtil.clamp(controlVolts, -12.0, 12.0);
    }
}
