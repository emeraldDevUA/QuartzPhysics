package org.tourmaline.Units;

import org.tourmaline.PlanePhysics.Tuple;

public class DoubleAbstractUnitConverter
        <U, V> extends AbstractUnitConverter<V>{

    public float convert(float unit,
                         Tuple<U, U> conversionType1,
                         Tuple<V, V> conversionType2){

        return 0;
    }



}
