package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public boolean[] shootersConnected = new boolean[0];
        public double shooterMotorsAverageVolts = 0.0;
        public double shooterMotorsTotalCurrentAmps = 0.0;
        public double[] shooterMotorsVelocityRPM = new double[0];

        public boolean[] feedersConnected = new boolean[0];
        public double feederMotorsAverageVolts = 0.0;
        public double feederMotorsTotalCurrentAmps = 0.0;
        public double[] feederMotorsVelocityRPM = new double[0];
    }

    /**
     * Updates the input values for the shooter subsystem.
     *
     * @param inputs the input object to update with current sensor values
     */
    void updateInputs(ShooterIOInputs inputs);

    /**
     * Sets the shooter motors to use open-loop voltage control.
     *
     * @param volts the voltage to apply to the shooter motors
     */
    default void setShooterMotorsVoltage(double volts) {}

    /**
     * Sets the feeder motors to use open-loop voltage control.
     *
     * @param volts the voltage to apply to the feeder motors
     */
    default void setFeederMotorsVoltage(double volts) {}

    /**
     * Sets the shooter motors to use closed-loop velocity control.
     *
     * @param rpm the target velocity in RPM for the shooter motors
     */
    default void setShooterVelocity(double rpm) {}
}
