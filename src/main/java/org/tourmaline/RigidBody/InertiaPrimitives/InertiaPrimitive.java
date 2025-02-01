package org.tourmaline.RigidBody.InertiaPrimitives;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.Setter;
import org.joml.Vector3f;

@AllArgsConstructor
@Getter
@Setter
public abstract class InertiaPrimitive {

    protected Vector3f size;
    protected Vector3f position;  // position in design coordinates
    protected Vector3f inertia;   // moment of inertia
    protected Vector3f offset;    // offset from center of gravity
    protected float mass;
    public float calculateVolume() { return size.x * size.y * size.z; }

    public Vector3f calculateInertia(){
        return null;
    }
}
