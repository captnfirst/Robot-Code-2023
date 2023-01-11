// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

//** Belirtilen saniye kadar bekler. Otonom komut grupları için kullanışlıdır! */
public class Wait extends CommandBase {

	double duration;
	Timer timer = new Timer();

	public Wait(double time) {
		duration = time;
	}

	public void initialize() {
		timer.reset();
		timer.start();
	}
	
	public boolean isFinished() {
		return timer.get() >= duration;
	}

	public void end(boolean interrupted) {
		timer.reset();
	}
}