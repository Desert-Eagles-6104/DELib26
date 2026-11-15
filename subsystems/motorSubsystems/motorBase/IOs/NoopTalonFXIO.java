// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib26.subsystems.motorSubsystems.motorBase.IOs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import frc.DELib26.motors.MotorIO;

public class NoopTalonFXIO implements MotorIO{
	public NoopTalonFXIO() {}

	@Override
	public void updateInputs(MotorIOInputs inputs){};

	@Override
	public void disableMotors(){};

	@Override
	public void resetPosition(double positionRotations){};

	@Override
	public void setMotionMagicPositionSetpoint(double positionRotations){};

	@Override
	public void setMotionMagicVelocity(double velocityRPS){};

	@Override
	public void runCharacterization(Voltage volts){};

	@Override
	public void setPositionSetpoint(double positionRotations){};

	@Override
	public void setVelocity(double velocityRPS){};

	@Override
	public void setPercentOutput(double percent, boolean ignoreSoftLimits){};

	@Override
	public void setPIDSlot(int slot){};

	@Override
	public void controlForwardSoftLimit(boolean enableSoftLimit, double forwardSoftLimitThreshold){};

	@Override
	public void controlReverseSoftLimit(boolean enableSoftLimit, double reverseSoftLimitThreshold){};

	@Override
	public void setEnableAutosetPositionValue(boolean forward, boolean reverse){};

	@Override
	public void setNeutralMode(NeutralModeValue mode){};

	@Override
	public void applyConfig(TalonFXConfiguration config){};
}
