package frc.robot.subsystems;

import static frc.robot.Constants.LEDs.*;

import com.techhounds.houndutil.houndlog.annotations.LoggedObject;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The LED subsystem performs only hardware initialization and continuously
 * clears
 * the LED buffer (turns all LEDs off) until new patterns are added.
 */
@LoggedObject
public class LEDs extends SubsystemBase {
    // Change depending on PWM slot
    private final AddressableLED leds = new AddressableLED(0);

    // Change depending on length in constants
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(0);

    private final Notifier updateNotifier;

    public LEDs() {
        // Change length of leds
        leds.setLength(0);
        clear();
        leds.setData(buffer);
        leds.start();

        updateNotifier = new Notifier(() -> {
            synchronized (this) {
                clear();
                leds.setData(buffer);
            }
        });
        updateNotifier.startPeriodic(0.02);

        setDefaultCommand(updateBufferCommand());
    }

    /**
     * Creates a command that continuously clears the LED buffer.
     *
     * @return the command that updates the LED buffer.
     */
    public Command updateBufferCommand() {
        return run(() -> {
            clear();
            leds.setData(buffer);
        }).ignoringDisable(true).withName("leds.updateBuffer");
    }

    public void clear() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, Color.kBlack);
        }
    }
}
