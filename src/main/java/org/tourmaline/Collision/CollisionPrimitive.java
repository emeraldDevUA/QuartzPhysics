package org.tourmaline.Collision;

import org.joml.Vector3f;

public abstract class CollisionPrimitive {
    private Vector3f position;

    public CollisionPrimitive(){

    }

    public abstract boolean checkCollision(CollisionPrimitive collisionObject);




}
