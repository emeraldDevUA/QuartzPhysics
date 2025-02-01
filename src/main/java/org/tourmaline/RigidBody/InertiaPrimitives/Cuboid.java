package org.tourmaline.RigidBody.InertiaPrimitives;

import org.joml.Vector3f;

import static java.lang.Math.pow;

public class Cuboid extends InertiaPrimitive {
    public Cuboid(Vector3f size, Vector3f position, Vector3f inertia, Vector3f offset, float mass) {
        super(size, position, inertia, offset, mass);
    }


    @Override
    public Vector3f getInertia() {

        float x = size.x, y = size.y, z = size.z;

        return new Vector3f(
                (float) (pow(y, 2) + pow(z, 2)),
                (float) (pow(x, 2) + pow(z, 2)),
                (float) (pow(x, 2) + pow(y, 2)))
                .mul(1.0f / 12.0f).mul(mass);
    }
}
