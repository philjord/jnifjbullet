package nifbullet;

import javax.media.j3d.Transform3D;

import utils.source.MeshSource;

import com.bulletphysics.collision.shapes.CollisionShape;

public interface PartedBulletNifModel extends BulletNifModel
{

	public void addPart(String partFileName, MeshSource meshSource, Object pointer, Transform3D rootTrans);

	public void setPartTransform(Object pointer, Transform3D t);

	public Object getPartPointer(CollisionShape collisionShape);

	public void removePart(Object pointer);

}
