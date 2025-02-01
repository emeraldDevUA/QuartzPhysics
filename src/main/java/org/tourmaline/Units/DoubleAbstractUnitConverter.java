package org.tourmaline.Units;

import org.tourmaline.PlanePhysics.Tuple;

public class DoubleAbstractUnitConverter<A, B> extends AbstractUnitConverter<B>{

    public float convert(float unit,
                         Tuple<A, A> conversionType1,
                         Tuple<B,B> conversionType2){

        return 0;
    }



}
