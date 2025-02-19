package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDs.LEDSection;

import static frc.robot.Constants.LEDs.*;
import static com.techhounds.houndutil.houndlib.leds.LEDPatterns.*;

@LoggedObject
public class LEDs extends SubsystemBase {
    private AddressableLED leds = new AddressableLED(PORT);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LENGTH);
    private final Notifier loadingNotifier;
    private ArrayList<LEDState> currentStates = new ArrayList<>();

    public enum LEDState {
        OFF(solid(Color.kBlack, LEDSection.ALL)),
        GOLD_BLUE_TRAIL(
                alternate(Color.kGold, Color.kBlue, 1, 255, LEDSection.ALL)),
        INITIALIZATION_BLACK_BACKGROUND(solid(Color.kBlack, LEDSection.ALL)),
        INITIALIZED_CONFIRM(breathe(Color.kGreen, 2, 0, 255, LEDSection.ALL));

        private List<Consumer<AddressableLEDBuffer>> bufferConsumers;

        @SafeVarargs
        private LEDState(Consumer<AddressableLEDBuffer>... bufferConsumer) {
            this.bufferConsumers = Arrays.asList(bufferConsumer);
        }
    }

    public LEDs() {
        leds.setLength(LENGTH);
        leds.setData(buffer);
        leds.start();

        loadingNotifier = new Notifier(
                () -> {
                    synchronized (this) {
                        LEDState.GOLD_BLUE_TRAIL.bufferConsumers.forEach(c -> c.accept(buffer));
                        leds.setData(buffer);
                    }
                });
        loadingNotifier.startPeriodic(0.02);

        setDefaultCommand(updateBufferCommand());
    }

    public Command requestStateCommand(LEDState state) {
        return Commands.run(() -> {
            if (!currentStates.contains(state)) {
                currentStates.add(state);
            }
        }).ignoringDisable(true);
    }

    public Command updateBufferCommand() {
        return run(() -> {
            loadingNotifier.stop();
            clear();

            if (!currentStates.contains(LEDState.GOLD_BLUE_TRAIL)) {
                currentStates.add(LEDState.GOLD_BLUE_TRAIL);
            }

            currentStates.sort((s1, s2) -> s2.ordinal() - s1.ordinal());
            currentStates.forEach(s -> s.bufferConsumers.forEach(c -> c.accept(buffer)));
            leds.setData(buffer);
            currentStates.clear();
        })
                .ignoringDisable(true)
                .withName("leds.updateBuffer");
    }

    public void clear() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kBlack);
        }
    }
}