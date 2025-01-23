package org.tourmaline.Collision;

import lombok.Getter;
import org.joml.Matrix3f;
import org.joml.Quaternionf;
import org.joml.Vector3f;

@Getter
public class BoundingBox extends CollisionPrimitive{
    private Vector3f halfDims;

    public BoundingBox(final Vector3f position, final Vector3f halfDims,
                       final Quaternionf quaternion ){
        setPosition(position);
        this.halfDims = halfDims;
        setQuaternion(quaternion);



    }


    private float getProjectionRadius(Vector3f axis, Matrix3f rotationMatrix) {
        // Радиус проекции это сумма проекций половинных размеров по всем осям
        Vector3f xAxis = new Vector3f();
        Vector3f yAxis = new Vector3f();
        Vector3f zAxis = new Vector3f();
        rotationMatrix.getColumn(0, xAxis);  // Получаем ось X коробки
        rotationMatrix.getColumn(1, yAxis);  // Получаем ось Y коробки
        rotationMatrix.getColumn(2, zAxis);  // Получаем ось Z коробки

        // Скалярные произведения осей с вектором для получения проекций
        return  halfDims.x * Math.abs(axis.dot(xAxis)) +
                halfDims.y * Math.abs(axis.dot(yAxis)) +
                halfDims.z * Math.abs(axis.dot(zAxis));
    }

    // Метод для проверки пересечения на одной оси
    private boolean testAxis(Vector3f axis, BoundingBox other, Vector3f t, Matrix3f rotationA, Matrix3f rotationB) {
        float ra = getProjectionRadius(axis, rotationA);
        float rb = other.getProjectionRadius(axis, rotationB);
        float distance = Math.abs(t.dot(axis));  // Расстояние между коробками вдоль оси

        return distance <= ra + rb;  // Если проекции пересекаются
    }

    public boolean checkCollision(CollisionPrimitive other){
        Matrix3f rotationA = getRotationMatrix();
        Matrix3f rotationB = other.getRotationMatrix();

        // Вектор между центрами коробок
        Vector3f t = new Vector3f(other.getPosition()).sub(this.getPosition());

        // Преобразуем в локальные координаты первой коробки
        t = new Vector3f(t.dot(rotationA.getColumn(0, new Vector3f())),
                t.dot(rotationA.getColumn(1, new Vector3f())),
                t.dot(rotationA.getColumn(2, new Vector3f())));

        // Оси для проверки (сначала оси обеих коробок)
        Vector3f[] axes = new Vector3f[15];

        // Оси коробки A
        rotationA.getColumn(0, axes[0] = new Vector3f());
        rotationA.getColumn(1, axes[1] = new Vector3f());
        rotationA.getColumn(2, axes[2] = new Vector3f());

        // Оси коробки B
        rotationB.getColumn(0, axes[3] = new Vector3f());
        rotationB.getColumn(1, axes[4] = new Vector3f());
        rotationB.getColumn(2, axes[5] = new Vector3f());

        // Перекрёстные оси (A_i x B_j)
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                axes[6 + i * 3 + j] = new Vector3f(axes[i]).cross(axes[3 + j]);
            }
        }

        // Проверяем все 15 осей
        for (Vector3f axis : axes) {
            if (axis.lengthSquared() < 1e-6) continue;  // Пропускаем почти нулевые оси

            if (!testAxis(axis, (BoundingBox) other, t, rotationA, rotationB)) {
                return false;  // Нашли разделяющую ось, столкновения нет
            }
        }

        // Если разделяющая ось не найдена, значит, коробки пересекаются
        return true;
    }


}
