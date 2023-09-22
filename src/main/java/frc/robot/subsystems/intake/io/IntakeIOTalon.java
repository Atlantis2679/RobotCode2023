package frc.robot.subsystems.intake.io;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Utils.fields.FieldsTable;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class IntakeIOTalon extends IntakeIO {
    private final TalonSRX motor = new TalonSRX(MOTOR_ID);

    public IntakeIOTalon(FieldsTable fieldsTable) {
        super(fieldsTable);
        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration(
                true,
                CURRENT_LIMIT_AMP,
                0,
                0);
        motor.configSupplyCurrentLimit(currentLimitConfiguration);
    }

    @Override
    public void setSpeed(double demand) {
        motor.set(ControlMode.PercentOutput, demand);
    }

}
