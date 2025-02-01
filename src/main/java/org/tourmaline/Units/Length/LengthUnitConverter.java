package org.tourmaline.Units.Length;


import org.tourmaline.PlanePhysics.Tuple;
import org.tourmaline.Units.AbstractUnitConverter;

public class LengthUnitConverter extends AbstractUnitConverter<LengthUnits> {

    public float convert(float unit, Tuple<LengthUnits, LengthUnits> conversionType){

        return unit * conversionType.a.getValue() / conversionType.b.getValue();

    }


}
