package com.bulletphysics.util;

import java.util.HashMap;
import java.util.Map;

/**
 * EXACT copy of JBullet com.bulletphysics.util.ObjectPool, but with no object pooling at all
 * The old one appears to be wildly incompatible with multi dynamics worlds and causes crazy pointer bugs everywhere
 * This one should over rule the orginal, be careful wehn deploying to jars
 * @author philip
 *
 */
@SuppressWarnings("all")
public class ObjectPool<T>
{
	private Class<T> cls;

	public ObjectPool(Class<T> cls)
	{
		this.cls = cls;
	}

	private T create()
	{
		try
		{
			return cls.newInstance();
		}
		catch (InstantiationException e)
		{
			throw new IllegalStateException(e);
		}
		catch (IllegalAccessException e)
		{
			throw new IllegalStateException(e);
		}
	}

	public T get()
	{
		return create();
	}

	/**
	 * 
	 * @param obj
	 */
	public void release(T obj)
	{
		// nothign to see here
	}

	////////////////////////////////////////////////////////////////////////////

	private static ThreadLocal<Map> threadLocal = new ThreadLocal<Map>()
	{
		@Override
		protected Map initialValue()
		{
			return new HashMap();
		}
	};

	@SuppressWarnings("unchecked")
	public static <T> ObjectPool<T> get(Class<T> cls)
	{
		Map<Class<T>, ObjectPool<T>> map = threadLocal.get();

		ObjectPool<T> pool =   map.get(cls);
		if (pool == null)
		{
			pool = new ObjectPool<T>(cls);
			map.put(cls, pool);
		}

		return pool;
	}

	public static void cleanCurrentThread()
	{
		threadLocal.remove();
	}

}
