package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonSRX motor = new TalonSRX(IntakeConstants.MOTOR_ID);

    public Intake() {
        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(
            true,
                IntakeConstants.CURRENT_LIMIT_AMP,
                0,
                0
        );
        motor.configSupplyCurrentLimit(currentLimitConfiguration);
    }

    public void setSpeed(double demand) {
        demand = MathUtil.clamp(demand, -1, 1);
        motor.set(ControlMode.PercentOutput, demand);
    }

    @Override
    public void periodic() { }
}
