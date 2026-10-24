/* RaptorMain - Main TeleOp OpMode for Raptor Series Bots

List of Controls

DRIVER A
	LEFT STICK (Y) - Move forward (+) and backward (-)
	LEFT STICK (X) - Strafe right (+) and left (-)
	RIGHT STICK (X) - Turn clockwise (+) and counterclockwise (-)

	X - Enable/disable verbose mode
	B - Globally cancel all macros

DRIVER B
	RIGHT TRIGGER - Run intake group forwards
	LEFT TRIGGER - Run intake group in reverse
	LEFT STICK (Y) - Run transfer forwards (+) and backwards (-)
	RIGHT STICK (Y) - Run transfer forwards (+) and backwards (-)

	DPAD UP - Increase shooter preset velocity
	DPAD DOWN - Decrease shooter preset velocity
	DPAD RIGHT - Increase shooter velocity (30 ticks/sec)
	DPAD LEFT - Decrease shooter velocity (30 ticks/sec)

	A - Enable/disable shooter
	B - Globally cancel all macros
 */

package org.firstinspires.ftc.teamcode.teleop.normal.runners;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.robot.ActionableRaptorRobot;

import lib8812.common.robot.IRobot;
import lib8812.common.teleop.ITeleOpRunner;

@Config
public class RaptorMainRunner extends ITeleOpRunner {
	public RaptorMainRunner(boolean blueTeam) {
		bot = new ActionableRaptorRobot(blueTeam);
		shooterMaxVelo = bot.SHOOTER_VELO_FOR_MID_SHOT;
	}

	final ActionableRaptorRobot bot;

	boolean shooterEnabled = false;
	boolean verbose = false;

	double shooterMaxVelo;

	void toggleShooterEnabled() {
		if (shooterMaxVelo < 0) {
			shooterMaxVelo = bot.SHOOTER_VELO_FOR_MID_SHOT;
		} else {
			shooterEnabled = !shooterEnabled;
		}

		bot.driverControl.setShooterOff();
	}

	void runShooter() {
		if (!shooterEnabled) {
			bot.driverControl.setShooterOff();
			return;
		}

		bot.driverControl.setShooterVelocity(shooterMaxVelo);
	}

	void incrementVeloPreset() {
		for (int i = 0; i < bot.SHOOTER_VELO_PRESETS.length - 1; i++) {
			if (shooterMaxVelo <= bot.SHOOTER_VELO_PRESETS[i]) {
				shooterMaxVelo = bot.SHOOTER_VELO_PRESETS[i + 1];
				return;
			}
		}

		shooterMaxVelo = bot.SHOOTER_VELO_PRESETS[bot.SHOOTER_VELO_PRESETS.length - 1];
	}

	void decrementVeloPreset() {
		for (int i = bot.SHOOTER_VELO_PRESETS.length - 2; i >= 0; i--) {
			if (shooterMaxVelo >= bot.SHOOTER_VELO_PRESETS[i + 1]) {
				shooterMaxVelo = bot.SHOOTER_VELO_PRESETS[i];
				return;
			}
		}

		shooterMaxVelo = bot.SHOOTER_VELO_PRESETS[0];
	}

	@Override
	protected void internalRun() {
		Runnable cancelMacros = () -> {
			actions.clear();
			bot.releaseAllDevices();
		};

		// GAMEPAD 1 ------------------------------------------------------------------------------------

		keybinder.bind("b").of(gamepad1).to(cancelMacros);
		keybinder.bind("x").of(gamepad1).to(() -> verbose = !verbose);

		// GAMEPAD 2 ------------------------------------------------------------------------------------

		keybinder.bind("dpad_up").of(gamepad2).to(this::incrementVeloPreset);
		keybinder.bind("dpad_down").of(gamepad2).to(this::decrementVeloPreset);

		keybinder.bind("a").of(gamepad2).to(this::toggleShooterEnabled);
		keybinder.bind("b").of(gamepad2).to(cancelMacros);
		keybinder.bind("x").of(gamepad2).to(() -> actions.scheduleAll(bot.shootThree(shooterMaxVelo)));

		keybinder.bind("dpad_left").of(gamepad2).to(() -> shooterMaxVelo = Math.max(0, shooterMaxVelo - 30));
		keybinder.bind("dpad_right").of(gamepad2).to(() -> shooterMaxVelo += 30);

		while (opModeIsActive()) {
			/* DRIVE CONTROL */

			Vector2d linearVel = new Vector2d(-gamepad1.inner.left_stick_y, -gamepad1.inner.left_stick_x);
			double angVel = -gamepad1.inner.right_stick_x;

			bot.driverControl.applyDrivePower(linearVel.x, linearVel.y, angVel);

			/* END DRIVE CONTROL */

			bot.driverControl.setIntakePower(gamepad2.inner.right_trigger - gamepad2.inner.left_trigger);
			bot.driverControl.setTransferVelocity((gamepad2.inner.right_stick_y + gamepad2.inner.left_stick_y) * bot.TRANSFER_MAX_VELOCITY);

			runShooter();

			bot.drive.localizer.update();

			keybinder.executeActions();
			actions.execute();

			if (verbose) {
				telemetry.addData(
						"wheels",
						"rightFront (%.2f) leftFront (%.2f) rightBack (%.2f) leftBack (%.2f)",
						bot.rightFront.getPower(),
						bot.leftFront.getPower(),
						bot.rightBack.getPower(),
						bot.leftBack.getPower()
				);

				telemetry.addData("linearVel set", "x (%.2f) y (%.2f)", linearVel.x, linearVel.y);
				telemetry.addData("angVel set", "%.2f", angVel);
			}

			telemetry.addData(
					"intake group",
					"power (%.2f)",
					bot.intake.getPower()
			);

			telemetry.addData(
					"transfer",
					"velo (%.2f) velo frac (%.2f)",
					bot.transfer.getVelocity(),
					bot.transfer.getVelocity() / bot.TRANSFER_MAX_VELOCITY
			);

			if (shooterEnabled) {
				double realTargetVelo = shooterMaxVelo;

				double maxError = Math.max(
						Math.abs(bot.shooterLeft.getVelocity() - realTargetVelo),
						Math.abs(bot.shooterRight.getVelocity() - realTargetVelo)
				);

				boolean canShoot = maxError <= 20;

				telemetry.addData(
						"shooter",
						"velo limit (%.2f) left velo (%.2f) right velo (%.2f) shot type (%s) %s",
						shooterMaxVelo,
						bot.shooterLeft.getVelocity(),
						bot.shooterRight.getVelocity(),
						canShoot ? "READY TO SHOOT" : ""
				);
			} else {
				telemetry.addData(
						"shooter",
						"disabled, velo limit (%.2f) shot type (%s)",
						shooterMaxVelo,
						bot.getShooterVelocityPresetLabel(shooterMaxVelo)
				);
			}

			Pose2d botPose = bot.drive.localizer.getPose();
			telemetry.addData("current position", "x: %.2f in, y: %.2f in, heading: %.2f deg", botPose.position.x, botPose.position.y, Math.toDegrees(botPose.heading.toDouble()));

			telemetry.update();
		}
	}

	@Override
	protected IRobot getBot() {
		return bot;
	}
}
