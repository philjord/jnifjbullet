package nifbullet.cha;

import javax.media.j3d.Transform3D;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import com.bulletphysics.dynamics.character.KinematicCharacterController3.CharacterPositionListener;

import nifbullet.BulletNifModel;

public interface NifBulletChar extends BulletNifModel
{
	public void setTransform(Quat4f q, Vector3f v);
	public void setTransform(Transform3D trans);
 
	public void setCharacterPositionListener(CharacterPositionListener listener);
}
