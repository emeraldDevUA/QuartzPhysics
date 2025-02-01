package org.tourmaline.Units.Length;

import lombok.Getter;

@Getter
public enum LengthUnits {
    MILLIMETER(0.001f),
    CENTIMETER(0.01f),
    METER(1f),
    KILOMETER(1000f),
    INCH(0.0254f),
    FOOT(0.3048f),
    YARD(0.9144f),
    MILE(1609.34f),
    NAUTICAL_MILE(1852f);
    // Getter method
    private final float value;

    LengthUnits(float value) {  // Constructor
        this.value = value;
    }

}