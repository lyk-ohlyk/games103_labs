using UnityEngine;
using System.Collections;
using System.Runtime.CompilerServices;

public class PBD_model: MonoBehaviour {

	float 		t= 0.0333f;
	float		damping= 0.99f;
	int[] 		E;
	float[] 	L;
	Vector3[] 	V;
	Vector3 gravity = new Vector3(0, -9.8f, 0);

    float friction_mu_t = 0.4f;                 // friction coefficient
    float restitution = 0.5f;                   // for collision

    int mesh_size = 21;

    // Use this for initialization
    void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;

		//Resize the mesh.
		int n = mesh_size;
		Vector3[] X  	= new Vector3[n*n];
		Vector2[] UV 	= new Vector2[n*n];
		int[] T	= new int[(n-1)*(n-1)*6];
		for(int j=0; j<n; j++)
		for(int i=0; i<n; i++)
		{
			X[j*n+i] =new Vector3(5-10.0f*i/(n-1), 0, 5-10.0f*j/(n-1));
			UV[j*n+i]=new Vector3(i/(n-1.0f), j/(n-1.0f));
		}
		int t=0;
		for(int j=0; j<n-1; j++)
		for(int i=0; i<n-1; i++)	
		{
			T[t*6+0]=j*n+i;
			T[t*6+1]=j*n+i+1;
			T[t*6+2]=(j+1)*n+i+1;
			T[t*6+3]=j*n+i;
			T[t*6+4]=(j+1)*n+i+1;
			T[t*6+5]=(j+1)*n+i;
			t++;
		}
		mesh.vertices	= X;
		mesh.triangles	= T;
		mesh.uv 		= UV;
		mesh.RecalculateNormals ();

		//Construct the original edge list
		int[] _E = new int[T.Length*2];
		for (int i=0; i<T.Length; i+=3) 
		{
			_E[i*2+0]=T[i+0];
			_E[i*2+1]=T[i+1];
			_E[i*2+2]=T[i+1];
			_E[i*2+3]=T[i+2];
			_E[i*2+4]=T[i+2];
			_E[i*2+5]=T[i+0];
		}
		//Reorder the original edge list
		for (int i=0; i<_E.Length; i+=2)
			if(_E[i] > _E[i + 1]) 
				Swap(ref _E[i], ref _E[i+1]);
		//Sort the original edge list using quicksort
		Quick_Sort (ref _E, 0, _E.Length/2-1);

		int e_number = 0;
		for (int i=0; i<_E.Length; i+=2)
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
				e_number++;

		E = new int[e_number * 2];
		for (int i=0, e=0; i<_E.Length; i+=2)
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
			{
				E[e*2+0]=_E [i + 0];
				E[e*2+1]=_E [i + 1];
				e++;
			}

		L = new float[E.Length/2];
		for (int e=0; e<E.Length/2; e++) 
		{
			int i = E[e*2+0];
			int j = E[e*2+1];
			L[e]=(X[i]-X[j]).magnitude;
		}

		V = new Vector3[X.Length];
		for (int i=0; i<X.Length; i++)
			V[i] = new Vector3 (0, 0, 0);
	}

	void Quick_Sort(ref int[] a, int l, int r)
	{
		int j;
		if(l<r)
		{
			j=Quick_Sort_Partition(ref a, l, r);
			Quick_Sort (ref a, l, j-1);
			Quick_Sort (ref a, j+1, r);
		}
	}

	int  Quick_Sort_Partition(ref int[] a, int l, int r)
	{
		int pivot_0, pivot_1, i, j;
		pivot_0 = a [l * 2 + 0];
		pivot_1 = a [l * 2 + 1];
		i = l;
		j = r + 1;
		while (true) 
		{
			do ++i; while( i<=r && (a[i*2]<pivot_0 || a[i*2]==pivot_0 && a[i*2+1]<=pivot_1));
			do --j; while(  a[j*2]>pivot_0 || a[j*2]==pivot_0 && a[j*2+1]> pivot_1);
			if(i>=j)	break;
			Swap(ref a[i*2], ref a[j*2]);
			Swap(ref a[i*2+1], ref a[j*2+1]);
		}
		Swap (ref a [l * 2 + 0], ref a [j * 2 + 0]);
		Swap (ref a [l * 2 + 1], ref a [j * 2 + 1]);
		return j;
	}

	void Swap(ref int a, ref int b)
	{
		int temp = a;
		a = b;
		b = temp;
	}

	void Strain_Limiting()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] vertices = mesh.vertices;

		Vector3[] sum_X = new Vector3[vertices.Length];
		int[] edge_counts = new int[vertices.Length];

        //Apply PBD here.
        // for every edge e connecting i and j, update the arrays:
		for (int e=0; e<E.Length/2; e++)
		{
            int i = E[e*2+0];
            int j = E[e*2+1];
            Vector3 x_ij = vertices[i] - vertices[j];
			Vector3 x_center = 0.5f * (vertices[i] + vertices[j]);
			Vector3 delta_p_ij = 0.5f * L[e] * x_ij.normalized;
			sum_X[i] += x_center + delta_p_ij;
			sum_X[j] += x_center - delta_p_ij;
            edge_counts[i]++;
			edge_counts[j]++;
        }

		float alpha = 0.2f;
		for (int i = 0; i < vertices.Length; i++)
		{
            if (i == 0 || i == mesh_size - 1) continue;		
			if (edge_counts[i] == 0) continue;
			V[i] += ((alpha * vertices[i] + sum_X[i]) / (alpha + edge_counts[i]) - vertices[i]) / t;
            vertices[i] = (alpha * vertices[i] + sum_X[i]) / (alpha + edge_counts[i]);
        }

		mesh.vertices = vertices;
	}


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    bool TestSphereCollide(Vector3 sphere_position, float sphere_radius, Vector3 p)
    {
        //Test if the sphere collide with the point.
        if ((p - sphere_position).magnitude < sphere_radius + 0.2f)
            return true;
        else
            return false;
    }


    void Collision_Handling()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] X = mesh.vertices;

        //Handle colllision.
        // get position of component sphere_motion
        GameObject sphere = GameObject.Find("Sphere");
        if (sphere == null) return;
        Vector3 sphere_position = sphere.transform.position;
        float sphere_radius = sphere.transform.localScale.x / 2.0f;

        for (int i = 0; i < X.Length; ++i)
        {
            Vector3 collide_normal = (X[i] - sphere_position).normalized;

            if (!TestSphereCollide(sphere_position, sphere_radius, X[i])) continue;
            X[i] = sphere_position + collide_normal * (sphere_radius + 0.2f);  // 更新顶点位置至不穿插
            if (Vector3.Dot(V[i], collide_normal) > 0) continue;  // 顶点速度与法向量同向，不碰撞
                                                                  // 计算 vi_new
            Vector3 vi_n = Vector3.Dot(V[i], collide_normal) * collide_normal;
            Vector3 vi_t = V[i] - vi_n;
            float a = Mathf.Max(1.0f - friction_mu_t * (1.0f + restitution) * vi_n.magnitude / vi_t.magnitude, 0.0f);
            Vector3 vi_n_new = -restitution * vi_n;
            Vector3 vi_t_new = a * vi_t;
            Vector3 vi_new = vi_n_new + vi_t_new;
            V[i] = vi_new;
        }
        mesh.vertices = X;
    }

    private void Reset()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] X = mesh.vertices;
        for (int i = 0; i < X.Length; ++i)
		{
			X[i] = new Vector3(5 - 10.0f * (i % mesh_size) / (mesh_size - 1), 0, 5 - 10.0f * (i / mesh_size) / (mesh_size - 1));
        }
		mesh.vertices = X;
		V = new Vector3[X.Length];
    }


    // Update is called once per frame
    void Update () 
	{
		// press R for Reset
		if (Input.GetKeyDown(KeyCode.R))
		{
            Reset();
        }

		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;

		for(int i=0; i<X.Length; i++)
		{
			if(i==0 || i== mesh_size - 1)	continue;
			//Initial Setup
			V[i] *= damping;
			V[i] += gravity * t;
			X[i] += V[i] * t;
		}
		mesh.vertices = X;

		for(int l=0; l<32; l++)
			Strain_Limiting ();

		Collision_Handling ();

		mesh.RecalculateNormals ();
	}
}

