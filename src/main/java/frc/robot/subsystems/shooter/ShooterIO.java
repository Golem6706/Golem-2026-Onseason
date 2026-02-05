package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        // Shooter Motors
        public boolean shooterMotor1Connected = false;
        public double shooterMotor1CurrentAmps = 0.0;
        public double shooterMotor1OutputVolts = 0.0;

        public boolean shooterMotor2Connected = false;
        public double shooterMotor2CurrentAmps = 0.0;
        public double shooterMotor2OutputVolts = 0.0;

        public boolean shooterMotor3Connected = false;
        public double shooterMotor3CurrentAmps = 0.0;
        public double shooterMotor3OutputVolts = 0.0;

        public boolean feederMotor1Connected = false;
        public double feederMotor1CurrentAmps = 0.0;
        public double feederMotor1OutputVolts = 0.0;

        public boolean feederMotor2Connected = false;
        public double feederMotor2CurrentAmps = 0.0;
        public double feederMotor2OutputVolts = 0.0;
    }

    void updateInputs(ShooterIOInputs inputs);

    default void setShooterMotorsVoltage(double volts) {}

    default void setFeederMotorsVoltage(double volts) {}
}
