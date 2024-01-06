using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using UnityEngine;


public class implicit_model : MonoBehaviour
{
	float 		t 		= 0.0333f;
	float 		mass	= 10;
	float		damping	= 0.99f;
	float 		rho		= 0.995f;
	float 		spring_k = 8000;
	int[] 		E;  // Edges for force calculation, edge = (E[2*i], E[2*i+1])
	float[] 	L;  // Length of edges
	Vector3[] 	V;  // Velocity of vertices

    float friction_mu_t = 0.4f;                 // friction coefficient
    float restitution = 0.5f;                   // for collision

	int mesh_size = 21;

	bool use_chebyshev_acceleration = false;

    // Start is called before the first frame update
    void Start()
    {
		Mesh mesh = GetComponent<MeshFilter> ().mesh;

		//Resize the mesh.
		int n = mesh_size;
		Vector3[] X  	= new Vector3[n*n];
		Vector2[] UV 	= new Vector2[n*n];

        int[] triangles	= new int[(n-1)*(n-1)*6];
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
			triangles[t*6+0]=j*n+i;
			triangles[t*6+1]=j*n+i+1;
			triangles[t*6+2]=(j+1)*n+i+1;
			triangles[t*6+3]=j*n+i;
			triangles[t*6+4]=(j+1)*n+i+1;
			triangles[t*6+5]=(j+1)*n+i;
			t++;
		}
		mesh.vertices=X;
		mesh.triangles=triangles;
		mesh.uv = UV;
		mesh.RecalculateNormals ();


		//Construct the original E (E:边的描述，如E[0],E[1]表示第一条边的两个顶点)
		int[] _E = new int[triangles.Length*2];
		for (int i=0; i<triangles.Length; i+=3) 
		{
			_E[i*2+0]=triangles[i+0];
			_E[i*2+1]=triangles[i+1];
			_E[i*2+2]=triangles[i+1];
			_E[i*2+3]=triangles[i+2];
			_E[i*2+4]=triangles[i+2];
			_E[i*2+5]=triangles[i+0];
		}
		//Reorder the original edge list
		for (int i=0; i<_E.Length; i+=2)
			if(_E[i] > _E[i + 1]) 
				Swap(ref _E[i], ref _E[i+1]);
		//Sort the original edge list using quicksort
		Quick_Sort (ref _E, 0, _E.Length/2-1);

		int e_number = 0;
		for (int i=0; i<_E.Length; i+=2)
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) // 去除重复边
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
			int v0 = E[e*2+0];
			int v1 = E[e*2+1];
			L[e]=(X[v0]-X[v1]).magnitude;
		}

		V = new Vector3[X.Length];
		for (int i=0; i<V.Length; i++)
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    bool TestSphereCollide(Vector3 sphere_position, float sphere_radius, Vector3 p)
	{
		//Test if the sphere collide with the point.
		if ((p - sphere_position).magnitude < sphere_radius)
            return true;
        else
            return false;
	}

    void Collision_Handling()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
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
			X[i] = sphere_position + collide_normal * (sphere_radius + 0.001f);  // 更新顶点位置至不穿插
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

	void Get_Gradient(Vector3[] X, Vector3[] X_hat, float t, Vector3[] G)
	{
		//Momentum and Gravity.
		for (int i = 0; i < X.Length; i++)
		{
			G[i] = new Vector3(0, 9.8f, 0) + (X[i] - X_hat[i]) / (t * t) * mass;
		}

		//Spring Force.
		for (int e = 0; e < E.Length / 2; e++)
		{
            int v0 = E[e * 2 + 0];
            int v1 = E[e * 2 + 1];
            Vector3 d = X[v0] - X[v1];
            float l = d.magnitude;
            Vector3 f = spring_k * (l - L[e]) * d / l;
			G[v0] += f;
			G[v1] -= f;
        }		
	}

    // Update is called once per frame
	void Update () 
	{
		// update Camera by wasd
		float h = 0.0f;
		float v = 0.0f;
		if (Input.GetKey(KeyCode.W)) v += 1.0f;
		if (Input.GetKey(KeyCode.S)) v -= 1.0f;
		if (Input.GetKey(KeyCode.A)) h -= 1.0f;
		if (Input.GetKey(KeyCode.D)) h += 1.0f;
		Camera.main.transform.Translate(new Vector3(h, 0, v) * 0.1f);

		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X 		= mesh.vertices;

		Vector3[] last_X 	= new Vector3[X.Length];
		Vector3[] X_hat 	= new Vector3[X.Length];
		Vector3[] G 		= new Vector3[X.Length];

        //Initial Setup. Damping the V.
        for (int i=0; i<X.Length; i++)
        {
            V[i] *= damping;
            last_X[i] = X[i];
			X_hat[i] = X[i] + V[i]*t;
        }

		use_chebyshev_acceleration = false;  // 老实说，不太会做

        float omega = 1.0f;
        // k 次 Newton's method 迭代
        for (int k=0; k<64; k++)
        {
            Get_Gradient(X, X_hat, t, G);
            if (!use_chebyshev_acceleration)
            {
                //Update X by gradient, by using a much simpler method by considering the Hessian as a diagonal matrix.
                for (int i = 0; i < X.Length; i++)
				{
					if (i == 0 || i == mesh_size - 1) continue;  // 顶点在边界，不更新
					X[i] -= G[i] / (mass / (t * t) + 4 * spring_k);
				}
			}
        }

        //Finishing.
		for (int i = 0; i < X.Length; ++i)
        {
            V[i] = (X[i] - last_X[i]) / t;
		}

        mesh.vertices = X;

		Collision_Handling ();
		mesh.RecalculateNormals ();
	}
}
