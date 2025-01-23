package org.tourmaline.Units;

import org.tourmaline.PlanePhysics.Tuple;

public abstract class AbstractUnitConverter{

    public float convert(float unit, Tuple<?, ?> conversionType){
        return 0;
    }


    public abstract AbstractUnitConverter getInstance();

}




