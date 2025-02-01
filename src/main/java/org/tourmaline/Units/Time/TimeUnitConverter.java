package org.tourmaline.Units.Time;

import org.tourmaline.PlanePhysics.Tuple;
import org.tourmaline.Units.AbstractUnitConverter;

public class TimeUnitConverter extends AbstractUnitConverter<TimeUnits> {

    public float convert(float unit, Tuple<TimeUnits, TimeUnits> conversionType){

        return unit * conversionType.a.getValue() / conversionType.b.getValue();

    }

}
