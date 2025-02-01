package org.tourmaline.Collision;

import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;
import org.joml.Matrix3f;
import org.joml.Quaternionf;
import org.joml.Vector3f;

@NoArgsConstructor
@Getter
@Setter
public abstract class CollisionPrimitive {
    protected float epsilon = 1;

    private Vector3f position;
    private Quaternionf quaternion;
    private Runnable collisionLambda;


    public CollisionPrimitive(Vector3f position){

        this.position = position;
    }

    public abstract boolean checkCollision(CollisionPrimitive collisionObject);


    protected Matrix3f getRotationMatrix() {
        Matrix3f rotation = new Matrix3f();
        getQuaternion().get(rotation);  // Преобразование кватерниона в матрицу 3x3
        return rotation;
    }

}
