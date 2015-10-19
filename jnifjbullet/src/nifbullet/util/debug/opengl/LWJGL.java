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

import java.awt.event.InputEvent;

import org.lwjgl.LWJGLException;
import org.lwjgl.input.Keyboard;
import org.lwjgl.input.Mouse;
import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.DisplayMode;
import org.lwjgl.opengl.PixelFormat;

/**
 *
 * @author jezek2
 */
public class LWJGL
{
	private static LwjglGL gl = new LwjglGL();

	private static String title;

	private static DemoApplication demoApp;

	private static boolean doRun = false;

	private static long lastTime = System.currentTimeMillis();

	private static int frames = 0;

	public static IGL getGL()
	{
		return gl;
	}

	/**
	 * inti and step MUST be on the same thread!!
	 * in fact ALL bullet adds removes, steps and display must be on the one thread
	 * @param width
	 * @param height
	 * @param title
	 * @param demoApp
	 * @throws LWJGLException
	 */
	public static void init(int width, int height, String title2, DemoApplication demoApp2) throws LWJGLException
	{
		LWJGL.title = title2;
		LWJGL.demoApp = demoApp2;

		Display.setDisplayMode(new DisplayMode(width, height));
		Display.setTitle(title);
		Display.create(new PixelFormat(0, 24, 0));

		Keyboard.create();
		Keyboard.enableRepeatEvents(true);
		Mouse.create();

		gl.init();

		demoApp.myinit();
		demoApp.reshape(width, height);
		doRun = true;
	}

	public static boolean isDoRun()
	{
		return doRun;
	}

	public static void setDoRun(boolean doRun)
	{
		LWJGL.doRun = doRun;
	}

	public static void exit()
	{
		doRun = false;
		Display.destroy();
	}

	public static void step()
	{
		if (doRun)
		{
			if (Display.isCloseRequested())
			{
				exit();
			}
			else
			{
				demoApp.moveAndDisplay();
				Display.update();

				int modifiers = 0;
				if (Keyboard.isKeyDown(Keyboard.KEY_LSHIFT) || Keyboard.isKeyDown(Keyboard.KEY_RSHIFT))
					modifiers |= InputEvent.SHIFT_DOWN_MASK;
				if (Keyboard.isKeyDown(Keyboard.KEY_LCONTROL) || Keyboard.isKeyDown(Keyboard.KEY_RCONTROL))
					modifiers |= InputEvent.CTRL_DOWN_MASK;
				if (Keyboard.isKeyDown(Keyboard.KEY_LMETA) || Keyboard.isKeyDown(Keyboard.KEY_RMETA))
					modifiers |= InputEvent.ALT_DOWN_MASK;

				while (Keyboard.next())
				{
					if (Keyboard.getEventCharacter() != '\0')
					{
						demoApp.keyboardCallback(Keyboard.getEventCharacter(), Mouse.getX(), Mouse.getY(), modifiers);
					}

					if (Keyboard.getEventKeyState())
					{
						demoApp.specialKeyboard(Keyboard.getEventKey(), Mouse.getX(), Mouse.getY(), modifiers);
					}
					else
					{
						demoApp.specialKeyboardUp(Keyboard.getEventKey(), Mouse.getX(), Mouse.getY(), modifiers);
					}

					if (Keyboard.getEventKey() == Keyboard.KEY_ESCAPE || Keyboard.getEventKey() == Keyboard.KEY_Q)
						//can't use J as initn and exit are caled at once	|| Keyboard.getEventKey() == Keyboard.KEY_J)
						exit();
				}

				while (Mouse.next())
				{
					if (Mouse.getEventButton() != -1)
					{
						int btn = Mouse.getEventButton();
						if (btn == 1)
						{
							btn = 2;
						}
						else if (btn == 2)
						{
							btn = 1;
						}
						demoApp.mouseFunc(btn, Mouse.getEventButtonState() ? 0 : 1, Mouse.getEventX(), Display.getDisplayMode().getHeight()
								- 1 - Mouse.getEventY());
					}
					demoApp.mouseMotionFunc(Mouse.getEventX(), Display.getDisplayMode().getHeight() - 1 - Mouse.getEventY());
				}

				long time = System.currentTimeMillis();
				if (time - lastTime < 1000)
				{
					frames++;
				}
				else
				{
					Display.setTitle(title + " | FPS: " + frames);
					lastTime = time;
					frames = 0;
				}
			}
		}

	}
}
