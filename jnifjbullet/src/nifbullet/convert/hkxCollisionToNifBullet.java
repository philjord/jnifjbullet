package nifbullet.convert;

import org.jogamp.java3d.utils.geometry.GeometryInfo;
import org.jogamp.vecmath.Point3f;
import org.jogamp.vecmath.Vector3f;

import com.bulletphysics.collision.shapes.BvhTriangleMeshShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.ConvexHullShape;
import com.bulletphysics.collision.shapes.IndexedMesh;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.collision.shapes.TriangleIndexVertexArray;
import com.bulletphysics.extras.gimpact.GImpactMeshShape;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;

import nif.NifVer;
import nif.niobject.hkx.hkAabb;
import nif.niobject.hkx.hkcdStaticMeshTreeBasePrimitive;
import nif.niobject.hkx.hknpCompressedMeshShapeData;
import nif.niobject.hkx.hknpCompressedMeshShapeTree;
import nif.niobject.hkx.hknpConvexPolytopeShape;
import nif.niobject.hkx.hknpSphereShape;
import nifbullet.util.NifBulletUtil;
import utils.convert.ConvertFromHavok;

/**
 * NOTE! compundShapes are expensive adn cause jitter possibly multi shper will be ok?
 * @author philip
 *
 */
public abstract class hkxCollisionToNifBullet
{
	public static CollisionShape hknpSphereShape(hknpSphereShape data, float scale, NifVer nifVer)
	{
		float radius = ConvertFromHavok.toJ3d(data.convexRadius, scale, nifVer);
		SphereShape s = new SphereShape(radius);
		return s;
	}

	
/*	public static CollisionShape hknpCapsuleShape(hknpCapsuleShape data, float scale, NifVer nifVer)
	{
		float radius = ConvertFromHavok.toJ3d(data.radius, scale, nifVer);
		Vector3f v1 = ConvertFromHavok.toJ3d(data.firstPoint, scale, nifVer);
		float radius1 = ConvertFromHavok.toJ3d(data.radius1, scale, nifVer);
		Vector3f v2 = ConvertFromHavok.toJ3d(data.secondPoint, scale, nifVer);
		float radius2 = ConvertFromHavok.toJ3d(data.radius2, scale, nifVer);

		if (radius != radius1 || radius != radius2)
		{
			System.out.println("bhkCapsuleShape radius != radius1 || radius != radius2 ");
		}

		float length = new Point3f(v2).distance(new Point3f(v1));

		Vector3f diff = new Vector3f(v2);

		diff.sub(v1);
		diff.scale(0.5f);
		v2.sub(diff);

		Transform3D t1 = new Transform3D();
		Transform3D t2 = new Transform3D();

		// note up must not be parallel to diff hence crazy up vector
		t1.lookAt(new Point3d(v2), new Point3d(v1), new Vector3d(diff.y, diff.z, diff.x));
		t1.invert();

		t2.rotX(Math.PI / 2);
		t1.mul(t2);

		CapsuleShape gcs = new CapsuleShape(radius, length);
		Matrix4f m = new Matrix4f();
		t1.get(m);
		Transform gt = new Transform(m);
		CompoundShape parent = new CompoundShape();
		parent.addChildShape(gt, gcs);

		return parent;
	}*/

	public static CollisionShape hknpConvexPolytopeShape(hknpConvexPolytopeShape data, float scale, NifVer nifVer)
	{
		ObjectArrayList<Vector3f> points = new ObjectArrayList<Vector3f>();

		if( data.vertices.length > 3 && data.planes.length > 3) { 
			for (int i = 0; i < 3; i++) {
				//System.out.println("i " + i + "" + ConvertFromHavok.toJ3dP3f(data.vertices[i], nifVer));
			}
			for (int i = 3; i < data.vertices.length; i++) {
				points.add(new Vector3f(ConvertFromHavok.toJ3dP3f(data.vertices[i], nifVer)));
			}
			for (int i = 0; i < data.planes.length - 3; i++) {
				points.add(new Vector3f(ConvertFromHavok.toJ3dP3f(data.planes[i], nifVer)));
			}
		} else {
			System.out.println("Interesting hknpConvexPolytopeShape " + data.vertices.length + " " + data.planes.length);
		}

		ConvexHullShape chs = new ConvexHullShape(points);

		return chs;
	}

	 

	public static CollisionShape hknpCompressedMeshShape(hknpCompressedMeshShapeData data, boolean isDynamic, float scale, NifVer nifVer)
	{
		
		TriangleIndexVertexArray indexVertexArrays = new TriangleIndexVertexArray();
		// we are in fact dealing only with the meshTree not the simdTree
		hknpCompressedMeshShapeTree meshTree = data.meshTree;
		
		// first decompress shared Vertices against the full AABB
		hkAabb meshTreehkAabb = meshTree.domain; // full AABB of all sections
			
		Point3f[] sharedVertices = new Point3f[0];// avoid null pointer checks
		if(meshTree.sharedVertices != null) {
			sharedVertices = new Point3f[meshTree.sharedVertices.length];
			for (int pvi = 0; pvi < meshTree.sharedVertices.length; pvi++) {			
				long pv = meshTree.sharedVertices[pvi];			
			
				//z value are what I'm calling x, so is this is in zyx format, 10, 11, 11 bit			 
				//21bit x, 21 bit y, 22 bit z this time!
				float fx = (((pv >> 0) & 0x1FFFFF)/(float)0x1FFFFF * (meshTreehkAabb.max.x-meshTreehkAabb.min.x)) + meshTreehkAabb.min.x;
				float fy = (((pv >> 21) & 0x1FFFFF)/(float)0x1FFFFF * (meshTreehkAabb.max.y-meshTreehkAabb.min.y)) + meshTreehkAabb.min.y;
				float fz = (((pv >> 42) & 0x3FFFFF)/(float)0x3FFFFF * (meshTreehkAabb.max.z-meshTreehkAabb.min.z)) + meshTreehkAabb.min.z;
			
				sharedVertices[pvi] = ConvertFromHavok.toJ3dP3f(fx, fy, fz, nifVer);	
			}		
		}
		
		for (int s = 0; s < meshTree.sections.length; s++) {			
			// the current value is the lowest byte and the previous in the next short(?) up	
			int primitivesCount = meshTree.sections[s].primitives.data & 0xff;
			int primitivesOffset = (meshTree.sections[s].primitives.data) >> 8 & 0xffff;
			//int sharedCount = meshTree.sections[s].sharedVertices.data & 0xff;
			int sharedOffset = (meshTree.sections[s].sharedVertices.data) >> 8 & 0xffff;
						
			
			int firstPackedVertex = meshTree.sections[s].firstPackedVertex;
			int numPackedVertices = meshTree.sections[s].numPackedVertices;
			//int numSharedIndices = meshTree.sections[s].numSharedIndices;			
			//hkAabb currentSectionhkAabb = meshTree.sections[currentSectionIdx].domain;	
			float[] codecParms = meshTree.sections[s].codecParms;// parallel Algebraic Recursive Multilevel Solvers?   
			
			Point3f[] vertices = new Point3f[numPackedVertices + sharedVertices.length];
			
			for(int pvi = 0; pvi < numPackedVertices; pvi++) {

				int pv = meshTree.packedVertices[firstPackedVertex + pvi];			
			
				//z value are what I'm calling x, so is this is in zyx format, 10, 11, 11 bit			
				
				// Normalized would look like this, but as the normalization dividor is accounted for in the param scale we don't need it
				//float fx = (pv & 0x7FF) / 2047.0f; // 11bit
				//float fy = ((pv >> 11) & 0x7FF) / 2047.0f; // 11bit
				//float fz = ((pv >> 22) & 0x3FF) / 1023.0f; // 10bit
				// multiplised by the param scale, and offset added 
				float fx = (((pv >> 0) & 0x7FF) * codecParms[3]) + codecParms[0];
				float fy = (((pv >> 11) & 0x7FF) * codecParms[4]) + codecParms[1];
				float fz = (((pv >> 22) & 0x3FF) * codecParms[5]) + codecParms[2];
				
				vertices[pvi] = ConvertFromHavok.toJ3dP3f(fx, fy, fz, nifVer);		
			}
						
			//TODO: all shared are copied to the end (if any), bit poor in efficiency, shorten later
			for(int i =0 ; i< sharedVertices.length;i++) {
				vertices[numPackedVertices+i] = sharedVertices[i];					 
			}		
			
			
			// bum have to precount so we can allocate a index array
			int pointCount = 0;
			for (int p = primitivesOffset; p < primitivesOffset + primitivesCount; p++) {		
				hkcdStaticMeshTreeBasePrimitive primitive = meshTree.primitives[p]; 
				int[] indices = primitive.indices;
				//quad?
				if(indices[2] != indices[3]) {
					pointCount += 6;
				} else {//just a tri					
					pointCount += 3;
				}
			}						
			
			int[] listPoints = new int[pointCount];
			int idx = 0;
			for (int i = 0; i < primitivesCount; i++) {				
				int p = primitivesOffset + i;
				hkcdStaticMeshTreeBasePrimitive primitive = meshTree.primitives[p]; 
				
				int[] indices = primitive.indices;	

			
				try {			 				
					int[] sharedVerticesIndex = meshTree.sharedVerticesIndex;	
					if(sharedVerticesIndex != null) {
						// if any of the indices go beyond numPackedVertices then the distance beyond
						// is used as an index into the sharedVerticesIndex, starting at the <sharedOffset> for this section
						// each section has numSharedIndices of the sharedVerticesIndex as it's own
						// the index found is then used as an index into the shared vertex area of the vertices, which for now is at the end starting at 
						// numPackedVertices, but when a single packed and shared vertex array is used will be at the end of all packed vertices
															 				
						//quad?
						if(indices[2] != indices[3]) {
							listPoints[idx++] = indices[0] < numPackedVertices ? indices[0] : sharedVerticesIndex[(indices[0]-numPackedVertices)+sharedOffset]+numPackedVertices;
							listPoints[idx++] = indices[1] < numPackedVertices ? indices[1] : sharedVerticesIndex[(indices[1]-numPackedVertices)+sharedOffset]+numPackedVertices;
							listPoints[idx++] = indices[2] < numPackedVertices ? indices[2] : sharedVerticesIndex[(indices[2]-numPackedVertices)+sharedOffset]+numPackedVertices;
							listPoints[idx++] = indices[2] < numPackedVertices ? indices[2] : sharedVerticesIndex[(indices[2]-numPackedVertices)+sharedOffset]+numPackedVertices;
							listPoints[idx++] = indices[3] < numPackedVertices ? indices[3] : sharedVerticesIndex[(indices[3]-numPackedVertices)+sharedOffset]+numPackedVertices;
							listPoints[idx++] = indices[0] < numPackedVertices ? indices[0] : sharedVerticesIndex[(indices[0]-numPackedVertices)+sharedOffset]+numPackedVertices;
						} else {//just a tri					
							listPoints[idx++] = indices[0] < numPackedVertices ? indices[0] : sharedVerticesIndex[(indices[0]-numPackedVertices)+sharedOffset]+numPackedVertices;
							listPoints[idx++] = indices[1] < numPackedVertices ? indices[1] : sharedVerticesIndex[(indices[1]-numPackedVertices)+sharedOffset]+numPackedVertices;
							listPoints[idx++] = indices[2] < numPackedVertices ? indices[2] : sharedVerticesIndex[(indices[2]-numPackedVertices)+sharedOffset]+numPackedVertices;
						}
					} else {
						//quad?
						//FIXME: what does an over sized index but no shared vertexs mean?
						if(indices[2] != indices[3]) {
							listPoints[idx++] = indices[0] < numPackedVertices ? indices[0] : 0;
							listPoints[idx++] = indices[1] < numPackedVertices ? indices[1] : 0;
							listPoints[idx++] = indices[2] < numPackedVertices ? indices[2] : 0;
							listPoints[idx++] = indices[2] < numPackedVertices ? indices[2] : 0;
							listPoints[idx++] = indices[3] < numPackedVertices ? indices[3] : 0;
							listPoints[idx++] = indices[0] < numPackedVertices ? indices[0] : 0;
						} else {//just a tri					
							listPoints[idx++] = indices[0] < numPackedVertices ? indices[0] : 0;
							listPoints[idx++] = indices[1] < numPackedVertices ? indices[1] : 0;
							listPoints[idx++] = indices[2] < numPackedVertices ? indices[2] : 0;
						}
					}	
				} catch(ArrayIndexOutOfBoundsException e) {
					System.out.println("hkxCollisionToNifBullet ArrayIndexOutOfBoundsException " + e.getMessage());
				}
			}	
			
			// no op transform
			Transform t = new Transform();
			t.setIdentity();
			
			
			GeometryInfo gi = new GeometryInfo(GeometryInfo.TRIANGLE_ARRAY);
			gi.setCoordinates(vertices);
			gi.setCoordinateIndices(listPoints);
			gi.setUseCoordIndexOnly(true);
			
			addMesh(indexVertexArrays, gi, t);
		}




		//NOTE!!!!!!!!!!!!!! there is no default CONCAVE CONCAVE collider so we MUST use GImpact or collision system throws null pointers!!
		if (isDynamic)
		{
			GImpactMeshShape trimesh = new GImpactMeshShape(indexVertexArrays);
			trimesh.updateBound();
			return trimesh;
		}
		else
		{
			BvhTriangleMeshShape trimesh = new BvhTriangleMeshShape(indexVertexArrays, true);
			return trimesh;
		}
	}

	private static void addMesh(TriangleIndexVertexArray indexVertexArrays, GeometryInfo gi, Transform t)
	{
		gi.convertToIndexedTriangles();

		int[] coordIndices = gi.getCoordinateIndices();
		coordIndices = NifBulletUtil.remove2dTriangles(coordIndices);

		Point3f[] points = gi.getCoordinates();
		// transform (bake in) all coords by t	
		NifBulletUtil.transformPoints(points, t);
		float[] coords = NifBulletUtil.makePrimitive(points);

		IndexedMesh mesh = new IndexedMesh();

		mesh.numTriangles = coordIndices.length / 3;
		mesh.triangleIndexBase = NifBulletUtil.convertToIndexBuffer(coordIndices);
		mesh.triangleIndexStride = 4 * 3;
		mesh.numVertices = coords.length / 3;
		mesh.vertexBase = NifBulletUtil.convertToByteBuffer(coords);
		mesh.vertexStride = 4 * 3;

		indexVertexArrays.addIndexedMesh(mesh);

	}
}
