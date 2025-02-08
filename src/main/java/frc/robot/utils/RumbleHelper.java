package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class RumbleHelper {
    private final XboxController controller;
    private final Timer timer = new Timer();
    private boolean isRumbling = false;
    private double rumbleDuration;
    private double intensity;
    private int rumbleCount;
    private int currentRumbleCount = 0;
    private final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);

    public RumbleHelper(XboxController controller) {
        this.controller = controller;
        scheduler.scheduleAtFixedRate(this::update, 0, 20, TimeUnit.MILLISECONDS); // Update every 20ms
    }
    /**
     * Rumbles the controller for a specified duration and number of times.
     *
     * @param duration The duration of each rumble in seconds.
     * @param count    The number of times to rumble.
     * @param intensity The intensity of the rumble (0.0 to 1.0).
     */
    public void rumbleForDuration(double duration, int count, double intensity) {
        this.rumbleDuration = duration;
        this.rumbleCount = count;
        this.currentRumbleCount = 0;
        this.intensity = MathUtil.clamp(intensity, 0, 1);
        startRumble();
    }

    private void startRumble() {
        controller.setRumble(XboxController.RumbleType.kBothRumble, intensity);
        timer.reset();
        timer.start();
        isRumbling = true;
    }

    private void stopRumble() {
        controller.setRumble(XboxController.RumbleType.kBothRumble, 0);
        timer.stop();
        isRumbling = false;
    }

    private void update() {
        if (isRumbling && timer.hasElapsed(rumbleDuration)) {
            stopRumble();
            currentRumbleCount++;
            if (currentRumbleCount < rumbleCount) {
                Timer.delay(0.1); // Short delay between rumbles
                startRumble();
            }
        }
    }

    public void shutdown() {
        scheduler.shutdown();
    }
}