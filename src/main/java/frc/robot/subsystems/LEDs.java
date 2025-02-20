package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import com.techhounds.houndutil.houndlib.leds.LEDPatterns;
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

        // If you dont like chase use wave
        GOLD_BLUE_CHASE(
                chase(new Color("0018F9"), new Color("#FF6E05"),
                        25, 10, 50, true, LEDSection.ALL)),

        BLUE_WAVE(
                wave(new Color("0018F9"), 25, 10, 50, 150, LEDSection.ALL)),

        SOLID_GREEN(
                solid(new Color("3BB143"), LEDSection.ALL));

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
                        LEDState.GOLD_BLUE_CHASE.bufferConsumers.forEach(c -> c.accept(buffer));
                        leds.setData(buffer);
                    }
                });
        loadingNotifier.startPeriodic(0.02);

        setDefaultCommand(updateBufferCommand());
    }

    public Command requestStateCommand(LEDState state) {
        return Commands.run(() -> currentStates.add(state)).ignoringDisable(true);
    }

    public Command updateBufferCommand() {
        return run(() -> {
            loadingNotifier.stop();
            clear();

            if (!currentStates.contains(LEDState.GOLD_BLUE_CHASE)) {
                currentStates.add(LEDState.GOLD_BLUE_CHASE);
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