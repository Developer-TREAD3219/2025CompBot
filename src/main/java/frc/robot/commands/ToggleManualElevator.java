package frc.robot.commands;

public class ToggleManualElevator {
    private boolean isManualMode = false;

    public void toggleManualMode() {
        isManualMode = !isManualMode;
    }
    public boolean isManualMode() {
        return isManualMode;
    }
    public void enableManuelMode() {
        isManualMode = true;
    }
}
