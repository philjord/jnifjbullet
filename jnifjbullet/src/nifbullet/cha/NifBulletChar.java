package nifbullet.cha;

import org.jogamp.java3d.Transform3D;
import org.jogamp.vecmath.Quat4f;
import org.jogamp.vecmath.Vector3f;

import com.bulletphysics.dynamics.character.KinematicCharacterController.CharacterPositionListener;

import nifbullet.BulletNifModel;

public interface NifBulletChar extends BulletNifModel
{
	public void setTransform(Quat4f q, Vector3f v);
	public void setTransform(Transform3D trans);
 
	public void setCharacterPositionListener(CharacterPositionListener listener);
}
