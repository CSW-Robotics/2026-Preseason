import static org.junit.jupiter.api.Assertions.assertEquals;
import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.hal.HAL;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.ResourceAccessMode;
import org.junit.jupiter.api.parallel.ResourceLock;

@ResourceLock(value = "SimState", mode = ResourceAccessMode.READ_WRITE)
public class CANcoderTest {
    final double SET_DELTA = 0.1;
    final int CONFIG_RETRY_COUNT = 5;

    CANcoder cancoder;

    @BeforeEach
    public void constructDevices() {
        assert HAL.initialize(500, 0);

        cancoder = new CANcoder(0);
    }

    @Test
    public void testPosSetter() {
        final double firstSet = 0;
        final double secondSet = 24;
        var posGetter = cancoder.getPosition();
        var cfg = cancoder.getConfigurator();

        retryConfigApply(()->cfg.setPosition(firstSet));
        System.out.println("First set: " + posGetter.waitForUpdate(1) + " vs " + firstSet);
        assertEquals(posGetter.getValue().in(Rotations), firstSet, SET_DELTA);
        retryConfigApply(()->cfg.setPosition(secondSet));
        System.out.println("Second set: " + posGetter.waitForUpdate(1) + " vs " + secondSet);
        assertEquals(posGetter.getValue().in(Rotations), secondSet, SET_DELTA);
    }

    private void retryConfigApply(Supplier<StatusCode> toApply) {
        StatusCode finalCode = StatusCode.StatusCodeNotInitialized;
        int triesLeftOver = CONFIG_RETRY_COUNT;
        do{
            finalCode = toApply.get();
        } while (!finalCode.isOK() && --triesLeftOver > 0);
        assert(finalCode.isOK());
    }
}