package org.tourmaline.PlanePhysics;

public class Atmosphere {

    public static float getAirTemperature(float altitude) {

        return 288.15f - 0.0065f * altitude;
    }

    /**
     * Calculates the air density at a given altitude.
     * Valid for altitudes between 0 and 11,000 meters.
     *
     * @param altitude Altitude in meters (0 <= altitude <= 11000).
     * @return Air density in kg/m^3.
     * @throws IllegalArgumentException if the altitude is out of range.
     */
    public static float getAirDensity(float altitude) {
        if (altitude < 0.0f || altitude > 11000.0f) {
            throw new IllegalArgumentException("Altitude must be between 0 and 11,000 meters.");
        }
        float temperature = getAirTemperature(altitude);
        float pressure = 101325.0f * (float) Math.pow(1 - 0.0065f * (altitude / 288.15f), 5.25f);
        return 0.00348f * (pressure / temperature);
    }
}
