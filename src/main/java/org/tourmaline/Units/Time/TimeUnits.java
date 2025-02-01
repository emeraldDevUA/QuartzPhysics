package org.tourmaline.Units.Time;

import lombok.Getter;

@Getter
public enum TimeUnits {
    MILLISECOND(0.001f),
    SECOND(1f),
    MINUTE(60f),
    HOUR(3600f),
    DAY(86400f),
    WEEK(604800f),
    MONTH(2_628_000f), // Approximate (30.44 days)
    YEAR(31_536_000f), // Common year (365 days)
    DECADE(315_360_000f), // 10 years
    CENTURY(3_153_600_000f); // 100 years

    private final float value;

    TimeUnits(float value) {
        this.value = value;
    }

}