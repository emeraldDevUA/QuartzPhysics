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


    public float convert(float unit, Tuple<LengthUnits, LengthUnits> conversionType1,
                         Tuple<TimeUnits,TimeUnits> conversionType2){

        return unit * lengthUnitConverter.convert(1, conversionType1)
                / timeUnitConverter.convert(1, conversionType2);

    }

}

