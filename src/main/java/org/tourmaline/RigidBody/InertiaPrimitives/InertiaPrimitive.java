package org.tourmaline.RigidBody.InertiaPrimitives;


import lombok.Getter;
import lombok.Setter;
import org.joml.Vector3f;

@Getter
@Setter
public abstract class InertiaPrimitive {

    protected Vector3f size;
    protected Vector3f position;  // position in design coordinates
    protected Vector3f offset;    // offset from center of gravity
    protected float mass;

    public InertiaPrimitive(Vector3f position, Vector3f size, float mass){
        this.position = position;
        this.size = size;
        this.mass = mass;
        this.offset = new Vector3f(0);

    }
    public float calculateVolume() { return size.x * size.y * size.z; }

    public Vector3f calculateInertia(){
        return null;
    }
}
