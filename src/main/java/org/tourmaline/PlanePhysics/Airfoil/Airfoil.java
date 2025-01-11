package org.tourmaline.PlanePhysics.Airfoil;

import org.joml.Vector3f;
import org.tourmaline.PlanePhysics.Tuple;

import java.util.ArrayList;
import java.util.List;

public class Airfoil {
    float min_alpha, max_alpha;
    private List<Vector3f> data;
    public float cl_max;

    public Airfoil(List<Vector3f> data)
    {
        cl_max = 0;
        this.data = data;

        min_alpha = data.getFirst().x;
        max_alpha = data.getLast().x;

        for (Vector3f vector3f: data) {
            if (vector3f.y > cl_max) cl_max = vector3f.y;
        }
    }




    public Tuple<Float, Float> sample(float alpha) {
        // Scale alpha to the index
        int i = (int) scale(alpha, min_alpha, max_alpha, 0, data.size() - 1);

        // Return Cl (y) and Cd (z) as a Tuple (Java does not have a built-in tuple, you can create your own or use a library)
        return new Tuple<>(data.get(i).y, data.get(i).z);
    }

    // Helper function to scale the alpha value to an index
    private float scale(float alpha, float minAlpha, float maxAlpha, float minIndex, float maxIndex) {
        alpha = Math.max(min_alpha, alpha);
        alpha = Math.min(max_alpha, alpha);

        return  minIndex + (maxIndex-minIndex)*(alpha - minAlpha)/(maxAlpha-minAlpha);
    }
    public static List<Vector3f> arrayToList(double[] array){
        List<Vector3f> arrayList = new ArrayList<>(array.length/3);
        for(int i = 0 ; i < array.length; i+=3) {
            arrayList.add(new Vector3f
                    ((float) array[i], (float) array[i+1], (float) array[i+2]));
        }
        return arrayList;
    }
}