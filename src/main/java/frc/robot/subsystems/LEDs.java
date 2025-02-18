package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static com.techhounds.houndutil.houndlib.leds.LEDPatterns.*;

/**
 * The LED subsystem, which controls the state of the LEDs by superimposing
 * requested LED states and continuously updates the LED's buffer. Other classes
 * can request specific LED states to be active, and they will be applied in
 * priority order.
 * 
 * This version adds a green LED state when HoundBrian zeros all subsystems.
 */
@LoggedObject
public class LEDs extends SubsystemBase {
    /** The LEDs. */
    private AddressableLED leds = new AddressableLED(9);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LENGTH);

    private final Notifier updateNotifier;

    private ArrayList<LEDState> currentStates = new ArrayList<>();

    private boolean overrideGreen = false;

    private double overrideEndTime = 0.0;

    /**
     * An enum of all possible LED states, including the green override when
     * HoundBrian zeros all subsystems.
     */
    public enum LEDState {
        OFF(solid(Color.kBlack, LEDSection.ALL)),
        SOLID_GREEN(solid(Color.kGreen, LEDSection.ALL)), // HoundBrian Green Effect
        RAINBOW_WAVE(
                waveRainbow(1, 30, 20, 100, 255, LEDSection.ALL));

        private final List<Consumer<AddressableLEDBuffer>> bufferConsumers;

        @SafeVarargs
        private LEDState(Consumer<AddressableLEDBuffer>... bufferConsumer) {
            this.bufferConsumers = Arrays.asList(bufferConsumer);
        }
    }

    /**
     * Initializes the LEDs.
     */
    public LEDs() {
        leds.setLength(LENGTH);
        leds.setData(buffer);
        leds.start();

        // Notifier that continuously updates the LED buffer
        updateNotifier = new Notifier(() -> {
            synchronized (this) {
                updateLEDPattern();
                leds.setData(buffer);
            }
        });
        updateNotifier.startPeriodic(0.02);

        setDefaultCommand(updateBufferCommand());
    }

    public Command requestStateCommand(LEDState state) {
        return Commands.run(() -> currentStates.add(state)).ignoringDisable(true);
    }

    /**
     * Updates the LED buffer with the currently active LED states.
     */
    private void updateLEDPattern() {
        if (overrideGreen) {
            if (Timer.getFPGATimestamp() < overrideEndTime) {
                LEDState.SOLID_GREEN.bufferConsumers.forEach(c -> c.accept(buffer));
                return;
            } else {
                overrideGreen = false;
            }
        }

        clear();
        currentStates.addAll(DEFAULT_STATES);
        currentStates.sort((s1, s2) -> s2.ordinal() - s1.ordinal());
        currentStates.forEach(s -> s.bufferConsumers.forEach(c -> c.accept(buffer)));
        currentStates.clear();
    }

    public void setGreenOverride(double seconds) {
        overrideGreen = true;
        overrideEndTime = Timer.getFPGATimestamp() + seconds;
    }

    public Command updateBufferCommand() {
        return run(() -> {
            updateLEDPattern();
            leds.setData(buffer);
        })
                .ignoringDisable(true)
                .withName("leds.updateBuffer");
    }

    /** Clears the buffer. */
    public void clear() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kBlack);
        }
    }
}
