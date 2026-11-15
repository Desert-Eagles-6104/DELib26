// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib26.subsystems.motorSubsystems.motorBase;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.DELib26.motors.MotorConstants;
import frc.DELib26.motors.MotorFactory;
import frc.DELib26.motors.MotorIO;
import frc.DELib26.subsystems.motorSubsystems.motorBase.IOs.NoopTalonFXIO;
import frc.DELib26.subsystems.motorSubsystems.motorBase.IOs.SimTalonFXIO;
import frc.DELib26.subsystems.motorSubsystems.motorBase.IOs.TalonFXIO;
import frc.DELib26.util.RunMode;

public class TalonFXIOFactory {
	public enum MotorIOType {
		REAL,
		SIM,
		NOOP
	}

	public static MotorIO buildMasterIO(MotorSubsystemConfiguration configuration, MotorIOType type) {
		return TalonFXIOFactory.create(
			type,
			configuration,
			configuration.master,
			MotorFactory.createTalonFX(configuration.master)
		);
	}

	public static MotorIO[] buildFollowerIOs(MotorSubsystemConfiguration configuration, MotorIOType type) {
		final int followerCount = configuration.slaves.map(a -> a.length).orElse(0);

		if (followerCount == 0) {
			return new MotorIO[0];
		}

		final MotorConstants[] slaves = configuration.slaves.orElseThrow();
		final TalonFX[] slaveTalons = MotorFactory.createSlaveTalonsAndApplyConfig(slaves, configuration.master);

		MotorIO[] ios = new MotorIO[followerCount];
		for (int i = 0; i < followerCount; i++) {
			ios[i] = TalonFXIOFactory.create(type, configuration, slaves[i], slaveTalons[i]);
		}

		return ios;
	}

	private static MotorIO create(
		MotorIOType type,
		MotorSubsystemConfiguration subsystemConfig,
		MotorConstants motorConstants,
		TalonFX motor
	) {
		return switch (type) {
			case REAL -> new TalonFXIO(
				subsystemConfig, motorConstants, motor
			);

			case SIM -> new SimTalonFXIO(
				subsystemConfig, motorConstants, motor
			);

			case NOOP -> new NoopTalonFXIO();
		};
	}

	public static MotorIOType getDefaultType() {
		return RunMode.isSim() ? MotorIOType.SIM : MotorIOType.REAL;
	}
}