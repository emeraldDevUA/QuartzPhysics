package org.tourmaline.RigidBody;

public interface DampingFunctions {


    float getAngularVelocityDamping(float dt);

    float getVelocityDamping(float dt);

    float getAccelerationDamping(float dt);

}
