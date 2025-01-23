package org.tourmaline.Processing;

import lombok.RequiredArgsConstructor;
import org.joml.Vector3f;
import org.tourmaline.Collision.BoundingBox;
import org.tourmaline.Collision.CollisionPrimitive;
import org.tourmaline.RigidBody.RigidBody;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import java.util.concurrent.TimeUnit;

@RequiredArgsConstructor
public class PhysicsProcessor extends Thread{
    private static final int workerCount = 8;
    private final List<RigidBody> rigidBodies;
    private final ExecutorService executor =
            Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());

    private final float dt = 0.001f;
    public boolean isRunning = true;



    @Override
    public void run(){
        long t1, t2;
        t1 = System.nanoTime();
        while(isRunning) {

            t2 = System.nanoTime();
            if (t2 - t1 >= 16_666_66) {
                for (RigidBody rb : rigidBodies) {
                    executor.submit(() -> {
                        rb.update(dt);
                        System.out.println(rb);
                    });
                    executor.submit(this::processBodyCollision);
                }

                t1 = t2;
            }
        }

        executor.shutdown();
        try {
            if (!executor.awaitTermination(5, TimeUnit.SECONDS)) {
                executor.shutdownNow(); // Force shutdown if not terminated
            }
        } catch (InterruptedException e) {
            executor.shutdownNow();
            Thread.currentThread().interrupt();
        }
    }



    private void processBodyCollision(){
            boolean[][] collisionMatrix = new boolean[rigidBodies.size()][rigidBodies.size()];
            for(int i = 0; i < rigidBodies.size(); i ++){
                for(int j = 0; j < rigidBodies.size(); j++){
                    if(!collisionMatrix[i][j]){
                        continue;
                    }
                    RigidBody firstBody = rigidBodies.get(i);
                    RigidBody secondBody = rigidBodies.get(j);

                    CollisionPrimitive firstCollision = firstBody.getCollisionPrimitive();
                    CollisionPrimitive secondCollision = secondBody.getCollisionPrimitive();

                    if(firstCollision instanceof BoundingBox bb1
                            && secondCollision instanceof BoundingBox bb2){

                        Vector3f tmp = bb1.getHalfDims();

                        // nasty approximation to cut out at least some excess collisions
                        float radius1 = (float) max(tmp.x, tmp.y, tmp.z);
                        tmp = bb2.getHalfDims();
                        float radius2 = (float) max(tmp.x, tmp.y, tmp.z);

                        if(bb1.getPosition().length() - bb2.getPosition().length() > radius1 + radius2){
                            collisionMatrix[i][j] = false;
                            collisionMatrix[j][i] = false;
                            continue;
                        }


                    }

                    collisionMatrix[i][j] =  firstCollision.checkCollision(secondCollision);
                    collisionMatrix[j][i] = collisionMatrix[i][j];
                    if(collisionMatrix[i][j]){
                        // execute lambda.
                        Runnable temp = rigidBodies.get(i).getCollisionPrimitive().getCollisionLambda();
                        if(temp != null){
                            executor.submit(temp);
                        }
                        Runnable temp2  = rigidBodies.get(j).getCollisionPrimitive().getCollisionLambda();
                        if(temp != null){
                            executor.submit(temp2);
                        }



                    }
                }
            }
    }


    private double max(double a, double b, double c){
        if(a >= b && a >= c){
            return a;
        }
        if(b >= a && b >= c){
            return b;
        }
        return c;
    }


    public void addRigidBody(RigidBody rigidBody) {
        rigidBodies.add(rigidBody);
    }
}


