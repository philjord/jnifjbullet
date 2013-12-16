package com.bulletphysics.util;

import java.lang.reflect.Array;
import java.util.HashMap;
import java.util.Map;

/**
 * EXACT copy of Jbullet without any pooling
 */
@SuppressWarnings("all")
public class ArrayPool<T>
{
	private Class componentType;

	/**
	 * Creates object pool.
	 * 
	 * @param componentType
	 */
	public ArrayPool(Class componentType)
	{
		this.componentType = componentType;
	}

	@SuppressWarnings("unchecked")
	private T create(int length)
	{
		return (T) Array.newInstance(componentType, length);
	}

	/**
	 * Returns array of exactly the same length as demanded, or create one if not
	 * present in the pool.
	 * 
	 * @param length
	 * @return array
	 */
	@SuppressWarnings("unchecked")
	public T getFixed(int length)
	{
		return create(length);
	}

	/**
	 * Returns array that has same or greater length, or create one if not present
	 * in the pool.
	 * 
	 * @param length the minimum length required
	 * @return array
	 */
	@SuppressWarnings("unchecked")
	public T getAtLeast(int length)
	{
		return create(length);
	}

	/**
	 * Releases array into object pool.
	 * 
	 * @param array previously obtained array from this pool
	 */
	@SuppressWarnings("unchecked")
	public void release(T array)
	{

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

	/**
	 * Returns per-thread array pool for given type, or create one if it doesn't exist.
	 * 
	 * @param cls type
	 * @return object pool
	 */
	@SuppressWarnings("unchecked")
	public static <T> ArrayPool<T> get(Class cls)
	{
		Map map = threadLocal.get();

		ArrayPool<T> pool = (ArrayPool<T>) map.get(cls);
		if (pool == null)
		{
			pool = new ArrayPool<T>(cls);
			map.put(cls, pool);
		}

		return pool;
	}

	public static void cleanCurrentThread()
	{
		threadLocal.remove();
	}

}
