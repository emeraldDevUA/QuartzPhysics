package org.tourmaline.PlanePhysics;

import org.joml.Vector3f;
import org.tourmaline.RigidBody.RigidBody;

public class Engine implements ActiveElement{

    private float activation;

    private float thrust;
    public Engine(float thrust){
        this.activation = 0f;
        this.thrust = thrust;

    }

    @Override
    public Vector3f getForce(RigidBody rb) {
        return new Vector3f(thrust*activation,0,0);
    }
}
