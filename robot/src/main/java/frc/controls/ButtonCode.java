package frc.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonCode {

	private Joystick OP_PAD_BUTTONS = new Joystick(0);
	private Joystick OP_PAD_SWITCHES = new Joystick(0);

	public enum Buttons {
		SCORE_SPEAKER(1),
		SCORE_AMP(2),
		SCORE_TRAP(3),
		SCORE_CLIMB(4),
		INTAKE_GROUND(5),
		INTAKE_SOURCE(6),
		INTAKE_TRAP_SOURCE(7),
		SIGNAL_CO_OP(8),
		SIGNAL_AMP(9),
		SOURCE_LANE_LEFT(10),
		SOURCE_LANE_CENTER(11),
		SOURCE_LANE_RIGHT(12),
		TEMPORARY_ARM_UP(13);

		private final int button;

		Buttons(int button) {
			this.button = button;
		}

		public int getButton() {
			return button;
		}
	}

	public enum Switches {
		ARM_UP(1);
	

	    private final int switch;

	    Switches(int switch) {
		this.switch = switch;
	}
	}

	public void getButton(Buttons button) {
		return new JoystickButton(_joystick, getButtonNumber(button))
	}

	public void getSwitch() {

	}
}