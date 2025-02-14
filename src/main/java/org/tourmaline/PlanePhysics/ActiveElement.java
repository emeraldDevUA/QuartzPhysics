package org.tourmaline.PlanePhysics;

import org.joml.Vector3f;
import org.tourmaline.RigidBody.RigidBody;

public interface ActiveElement {

    void getForce(RigidBody rigidBody);

}
