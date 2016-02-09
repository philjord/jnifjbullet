/*
 * Java port of Bullet (c) 2008 Martin Dvorak <jezek2@advel.cz>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package nifbullet.util.debug.opengl;

import static nifbullet.util.debug.opengl.IGL.GL_COLOR_BUFFER_BIT;
import static nifbullet.util.debug.opengl.IGL.GL_DEPTH_BUFFER_BIT;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import nifbullet.util.NifBulletUtil;

import org.lwjgl.LWJGLException;
import org.lwjgl.input.Keyboard;

import tools3d.navigation.AvatarLocation;

import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.linearmath.QuaternionUtil;
import com.bulletphysics.linearmath.Transform;

public class DebugOutput extends DemoApplication
{

	private int gForward = 0;

	private int gBackward = 0;

	private int gLeft = 0;

	private int gRight = 0;

	private Transform cameraWorldTrans = NifBulletUtil.newIdentityTransform();

	// Eitehr manual use of the above variables or automatic following of the avatarlocation

	private AvatarLocation avatarLocation;

	private static DebugOutput demo;

	public static void initDebug(DynamicsWorld dw)
	{
		initDebug(dw, null);
	}

	public static void initDebug(DynamicsWorld dw, AvatarLocation al)
	{
		demo = new DebugOutput(LWJGL.getGL(), dw, al);
		demo.initPhysics();
		demo.getDynamicsWorld().setDebugDrawer(new GLDebugDrawer(LWJGL.getGL()));

		try
		{
			LWJGL.init(800, 600, "Physics Debug output", demo);
		}
		catch (LWJGLException e)
		{
			e.printStackTrace();
		}
	}
	
	public static void disposeDebug()
	{
		LWJGL.exit();
	}

	public DebugOutput(IGL gl, DynamicsWorld dw, AvatarLocation al)
	{
		super(gl);
		dynamicsWorld = dw;
		this.avatarLocation = al;
		cameraWorldTrans.setIdentity();
	}

	@Override
	public void shootBox(Vector3f destination)
	{
		//disabled because we are on a differetn thread from stepSIm
	}

	public void initPhysics()
	{

		Transform startTransform = NifBulletUtil.newIdentityTransform();
		startTransform.origin.set(0.0f, 4.0f, 0.0f);

		clientResetScene();

		setCameraDistance(56f);
	}

	@Override
	public void clientMoveAndDisplay()
	{
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// do walking if we aren't attached to a character
		if (avatarLocation == null)
		{
			float dt = getDeltaTimeMicroseconds() * 0.000001f;
			// set walkDirection for our character
			Transform xform = new Transform(cameraWorldTrans);

			Vector3f forwardDir = new Vector3f();
			xform.basis.getRow(2, forwardDir);
			//printf("forwardDir=%f,%f,%f\n",forwardDir[0],forwardDir[1],forwardDir[2]);
			Vector3f upDir = new Vector3f();
			xform.basis.getRow(1, upDir);
			Vector3f strafeDir = new Vector3f();
			xform.basis.getRow(0, strafeDir);
			forwardDir.normalize();
			upDir.normalize();
			strafeDir.normalize();

			Vector3f walkDirection = new Vector3f(0.0f, 0.0f, 0.0f);
			float walkVelocity = 1.1f * 60f; // 4 km/h -> 1.1 m/s
			float walkSpeed = walkVelocity * dt;

			if (gLeft != 0)
			{
				walkDirection.add(strafeDir);
			}

			if (gRight != 0)
			{
				walkDirection.sub(strafeDir);
			}

			if (gForward != 0)
			{
				walkDirection.add(forwardDir);
			}

			if (gBackward != 0)
			{
				walkDirection.sub(forwardDir);
			}

			walkDirection.scale(walkSpeed);
			cameraWorldTrans.origin.add(walkDirection);

		}

		if (dynamicsWorld != null)
		{
			// optional but useful: debug drawing
			dynamicsWorld.debugDrawWorld();
		}

		renderme();

		//glFlush();
		//glutSwapBuffers();

	}

	@Override
	public void displayCallback()
	{
		gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		renderme();

		if (dynamicsWorld != null)
		{
			dynamicsWorld.debugDrawWorld();
		}

		//glFlush();
		//glutSwapBuffers();
	}

	@Override
	public void clientResetScene()
	{

	}

	@Override
	public void specialKeyboardUp(int key, int x, int y, int modifiers)
	{
		switch (key)
		{
			case Keyboard.KEY_UP:
			{
				gForward = 0;
				break;
			}
			case Keyboard.KEY_DOWN:
			{
				gBackward = 0;
				break;
			}
			case Keyboard.KEY_LEFT:
			{
				gLeft = 0;
				break;
			}
			case Keyboard.KEY_RIGHT:
			{
				gRight = 0;
				break;
			}

			default:
				super.specialKeyboardUp(key, x, y, modifiers);
				break;
		}
	}

	@Override
	public void specialKeyboard(int key, int x, int y, int modifiers)
	{
		switch (key)
		{
			case Keyboard.KEY_UP:
			{
				gForward = 1;
				break;
			}
			case Keyboard.KEY_DOWN:
			{
				gBackward = 1;
				break;
			}
			case Keyboard.KEY_LEFT:
			{
				gLeft = 1;
				break;
			}
			case Keyboard.KEY_RIGHT:
			{
				gRight = 1;
				break;
			}

			default:
				super.specialKeyboard(key, x, y, modifiers);
				break;
		}
	}

	@Override
	public void updateCamera()
	{
		gl.glMatrixMode(IGL.GL_PROJECTION);
		gl.glLoadIdentity();

		if (avatarLocation == null)
		{
			cameraTargetPosition.set(cameraWorldTrans.origin);

			float rele = ele * 0.01745329251994329547f; // rads per deg
			float razi = azi * 0.01745329251994329547f; // rads per deg

			Quat4f rot = new Quat4f(0f, 0f, 0f, 1f);
			QuaternionUtil.setRotation(rot, cameraUp, razi);

			Vector3f eyePos = new Vector3f(0f, 0f, -1f);
			Vector3f right = new Vector3f(1, 0, 0);
			Quat4f roll = new Quat4f(0f, 0f, 0f, 1f);
			QuaternionUtil.setRotation(roll, right, -rele);

			Matrix3f tmpMat1 = new Matrix3f();
			Matrix3f tmpMat2 = new Matrix3f();
			tmpMat1.set(rot);
			tmpMat2.set(roll);
			tmpMat1.mul(tmpMat2);
			tmpMat1.transform(eyePos);
			eyePos.add(cameraTargetPosition);

			cameraPosition.set(eyePos);

			//	Vector3f pos = new Vector3f(0, 2.2f, 2f);// back and up a bit 

			//cameraWorldTrans.transform(pos);
			//cameraPosition.set(pos);
		}
		else
		{
			Transform3D avTrans = avatarLocation.getTransform();
			Transform characterWorldTrans = NifBulletUtil.createTrans(avTrans);

			cameraTargetPosition.set(characterWorldTrans.origin);
			cameraTargetPosition.y += 0.85;// look at head
			Vector3f pos = new Vector3f(0, 2.2f, 2f);// back and up a bit 
			Transform temp = NifBulletUtil.createTrans(avTrans);
			temp.transform(pos);
			cameraPosition.set(pos);
		}

		// update OpenGL camera settings
		gl.glFrustum(-1.0, 1.0, -1.0, 1.0, 1.0, 10000.0);

		gl.glMatrixMode(IGL.GL_MODELVIEW);
		gl.glLoadIdentity();

		gl.gluLookAt(cameraPosition.x, cameraPosition.y, cameraPosition.z, cameraTargetPosition.x, cameraTargetPosition.y,
				cameraTargetPosition.z, cameraUp.x, cameraUp.y, cameraUp.z);
	}

}
