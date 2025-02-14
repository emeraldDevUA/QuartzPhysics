package org.tourmaline.Units.Speed;

import org.tourmaline.PlanePhysics.Tuple;
import org.tourmaline.Units.DoubleAbstractUnitConverter;
import org.tourmaline.Units.Length.LengthUnitConverter;
import org.tourmaline.Units.Length.LengthUnits;
import org.tourmaline.Units.Time.TimeUnitConverter;
import org.tourmaline.Units.Time.TimeUnits;

public class SpeedUnitConverter
        extends DoubleAbstractUnitConverter<LengthUnits, TimeUnits> {

    private static final LengthUnitConverter lengthUnitConverter
            = new LengthUnitConverter();
    private static final TimeUnitConverter timeUnitConverter
            = new TimeUnitConverter();


    public float convert(float unit, Tuple<LengthUnits, TimeUnits> conversionType1,
                         Tuple<LengthUnits,TimeUnits> conversionType2){

        return unit * lengthUnitConverter.convert(1, new Tuple<>(conversionType1.a, conversionType2.a))
                / timeUnitConverter.convert(1, new Tuple<>(conversionType1.b, conversionType2.b));

    }

}

