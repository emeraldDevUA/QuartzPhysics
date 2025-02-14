import org.junit.jupiter.api.Test;
import org.tourmaline.PlanePhysics.Tuple;
import org.tourmaline.Units.Length.LengthUnits;
import org.tourmaline.Units.Speed.SpeedUnitConverter;
import org.tourmaline.Units.Time.TimeUnits;

import static org.junit.jupiter.api.Assertions.assertEquals;

class SpeedUnitConverterTest {

    private final SpeedUnitConverter speedConverter = new SpeedUnitConverter();
    private static final float DELTA = 1E-5f; // Precision tolerance

    @Test
    void testMetersPerSecondToKilometersPerHour() {
        float result = speedConverter.convert(
                1.0f,
                new Tuple<>(LengthUnits.METER, TimeUnits.SECOND), // Correct Tuple with Length and Time
                new Tuple<>(LengthUnits.KILOMETER, TimeUnits.HOUR) // Correct Tuple with Length and Time
        );
        assertEquals(3.6f, result, DELTA);
    }

    @Test
    void testKilometersPerHourToMetersPerSecond() {
        float result = speedConverter.convert(
                1.0f,
                new Tuple<>(LengthUnits.KILOMETER, TimeUnits.HOUR), // Correct Tuple with Length and Time
                new Tuple<>(LengthUnits.METER, TimeUnits.SECOND) // Correct Tuple with Length and Time
        );
        assertEquals(1.0f / 3.6f, result, DELTA);
    }

    @Test
    void testMilesPerHourToMetersPerSecond() {
        float result = speedConverter.convert(
                1.0f,
                new Tuple<>(LengthUnits.MILE, TimeUnits.HOUR), // Correct Tuple with Length and Time
                new Tuple<>(LengthUnits.METER, TimeUnits.SECOND) // Correct Tuple with Length and Time
        );
        assertEquals(0.44704f, result, DELTA);
    }

    @Test
    void testFeetPerSecondToKilometersPerHour() {
        float result = speedConverter.convert(
                1.0f,
                new Tuple<>(LengthUnits.FOOT, TimeUnits.SECOND), // Correct Tuple with Length and Time
                new Tuple<>(LengthUnits.KILOMETER, TimeUnits.HOUR) // Correct Tuple with Length and Time
        );
        assertEquals(1.09728f, result, DELTA);
    }

    @Test
    void testIdentityConversion() {
        float result = speedConverter.convert(
                1.0f,
                new Tuple<>(LengthUnits.METER, TimeUnits.SECOND), // Correct Tuple with Length and Time
                new Tuple<>(LengthUnits.METER, TimeUnits.SECOND) // Correct Tuple with Length and Time
        );
        assertEquals(1.0f, result, DELTA);
    }
}
