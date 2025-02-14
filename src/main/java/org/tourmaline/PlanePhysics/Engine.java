package org.tourmaline.PlanePhysics;

import lombok.Getter;
import lombok.Setter;
import org.joml.Vector3f;
import org.tourmaline.RigidBody.RigidBody;

@Setter
@Getter
public class Engine implements ActiveElement{

    private float activation;
    private float thrust;
    private boolean afterBurner;

    public Engine(float thrust){
        this.activation = 0f;
        this.thrust = thrust;

    }

    @Override
    public void getForce(RigidBody rigidBody) {
        rigidBody.applyForceAtPoint(
                new Vector3f(thrust*activation*(afterBurner?1:1.5f),0,0),
                new Vector3f(0));

    }
}
