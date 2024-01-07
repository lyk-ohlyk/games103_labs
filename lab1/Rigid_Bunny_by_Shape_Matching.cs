using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using UnityEngine;
using static UnityEditor.PlayerSettings;

public class Rigid_Bunny_by_Shape_Matching : MonoBehaviour
{
    public bool launched = false;
    Vector3 reset_c = new Vector3(0, 1f, 0f);
    Vector3[] X;  // current particals position
    Vector3[] Q;  // reference shape to center of mass (r_{i})
    Vector3[] V;  // velocity
    Matrix4x4 QQt = Matrix4x4.zero;  // Q * Q^t
    Vector3 gravity = new Vector3(0, -9.8f, 0);

    Vector3[] planes = new Vector3[2];
    Vector3[] N = new Vector3[2];

    float dt = 0.015f;
    float mu = 0.5f;
    float restitution = 0.5f;
    float linear_decay = 0.999f;                // for velocity decay


    // Start is called before the first frame update
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        V = new Vector3[mesh.vertices.Length];
        X = mesh.vertices;
        Q = mesh.vertices;

        print("Start X is:" + X.Length);

        //Centerizing Q.
        Vector3 c = Vector3.zero;
        for (int i = 0; i < Q.Length; i++)
        {
            c += Q[i];
        }

        c /= Q.Length;
        for (int i = 0; i < Q.Length; i++)
            Q[i] -= c;

        //Get QQ^t ready.
        for (int i = 0; i < Q.Length; i++)
        {
            QQt[0, 0] += Q[i][0] * Q[i][0];
            QQt[0, 1] += Q[i][0] * Q[i][1];
            QQt[0, 2] += Q[i][0] * Q[i][2];
            QQt[1, 0] += Q[i][1] * Q[i][0];
            QQt[1, 1] += Q[i][1] * Q[i][1];
            QQt[1, 2] += Q[i][1] * Q[i][2];
            QQt[2, 0] += Q[i][2] * Q[i][0];
            QQt[2, 1] += Q[i][2] * Q[i][1];
            QQt[2, 2] += Q[i][2] * Q[i][2];
        }
        QQt[3, 3] = 1;

        for (int i = 0; i < X.Length; i++)
            V[i][0] = 4.0f;

        Update_Mesh(transform.position, Matrix4x4.Rotate(transform.rotation), 0);
        transform.position = Vector3.zero;
        transform.rotation = Quaternion.identity;

        // Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
        // Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));
        planes[0] = new Vector3(0, 0.01f, 0);
        planes[1] = new Vector3(2, 0, 0);
        N[0] = new Vector3(0, 1, 0);
        N[1] = new Vector3(-1, 0, 0);
    }

    // Polar Decomposition that returns the rotation from F.
    Matrix4x4 Get_Rotation(Matrix4x4 F)
    {
        Matrix4x4 C = Matrix4x4.zero;
        for (int ii = 0; ii < 3; ii++)
            for (int jj = 0; jj < 3; jj++)
                for (int kk = 0; kk < 3; kk++)
                    C[ii, jj] += F[kk, ii] * F[kk, jj];

        Matrix4x4 C2 = Matrix4x4.zero;
        for (int ii = 0; ii < 3; ii++)
            for (int jj = 0; jj < 3; jj++)
                for (int kk = 0; kk < 3; kk++)
                    C2[ii, jj] += C[ii, kk] * C[jj, kk];

        float det = F[0, 0] * F[1, 1] * F[2, 2] +
                        F[0, 1] * F[1, 2] * F[2, 0] +
                        F[1, 0] * F[2, 1] * F[0, 2] -
                        F[0, 2] * F[1, 1] * F[2, 0] -
                        F[0, 1] * F[1, 0] * F[2, 2] -
                        F[0, 0] * F[1, 2] * F[2, 1];

        float I_c = C[0, 0] + C[1, 1] + C[2, 2];
        float I_c2 = I_c * I_c;
        float II_c = 0.5f * (I_c2 - C2[0, 0] - C2[1, 1] - C2[2, 2]);
        float III_c = det * det;
        float k = I_c2 - 3 * II_c;

        Matrix4x4 inv_U = Matrix4x4.zero;
        if (k < 1e-10f)
        {
            float inv_lambda = 1 / Mathf.Sqrt(I_c / 3);
            inv_U[0, 0] = inv_lambda;
            inv_U[1, 1] = inv_lambda;
            inv_U[2, 2] = inv_lambda;
        }
        else
        {
            float l = I_c * (I_c * I_c - 4.5f * II_c) + 13.5f * III_c;
            float k_root = Mathf.Sqrt(k);
            float value = l / (k * k_root);
            if (value < -1.0f) value = -1.0f;
            if (value > 1.0f) value = 1.0f;
            float phi = Mathf.Acos(value);
            float lambda2 = (I_c + 2 * k_root * Mathf.Cos(phi / 3)) / 3.0f;
            float lambda = Mathf.Sqrt(lambda2);

            float III_u = Mathf.Sqrt(III_c);
            if (det < 0) III_u = -III_u;
            float I_u = lambda + Mathf.Sqrt(-lambda2 + I_c + 2 * III_u / lambda);
            float II_u = (I_u * I_u - I_c) * 0.5f;


            float inv_rate, factor;
            inv_rate = 1 / (I_u * II_u - III_u);
            factor = I_u * III_u * inv_rate;

            Matrix4x4 U = Matrix4x4.zero;
            U[0, 0] = factor;
            U[1, 1] = factor;
            U[2, 2] = factor;

            factor = (I_u * I_u - II_u) * inv_rate;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    U[i, j] += factor * C[i, j] - inv_rate * C2[i, j];

            inv_rate = 1 / III_u;
            factor = II_u * inv_rate;
            inv_U[0, 0] = factor;
            inv_U[1, 1] = factor;
            inv_U[2, 2] = factor;

            factor = -I_u * inv_rate;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    inv_U[i, j] += factor * U[i, j] + inv_rate * C[i, j];
        }

        Matrix4x4 R = Matrix4x4.zero;
        for (int ii = 0; ii < 3; ii++)
            for (int jj = 0; jj < 3; jj++)
                for (int kk = 0; kk < 3; kk++)
                    R[ii, jj] += F[ii, kk] * inv_U[kk, jj];
        R[3, 3] = 1;
        return R;
    }

    // Update the mesh vertices according to translation c and rotation R.
    // It also updates the velocity.
    void Update_Mesh(Vector3 c, Matrix4x4 R, float inv_dt)
    {
        for (int i = 0; i < Q.Length; i++)
        {
            Vector3 x = (Vector3)(R * Q[i]) + c;

            V[i] += (x - X[i]) * inv_dt;
            X[i] = x;
        }
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        mesh.vertices = X;
        mesh.RecalculateNormals();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    bool TestCollision(Vector3 t, Vector3 _plane, Vector3 _normal)
    {
        return Vector3.Dot(t - _plane, _normal) < 0;
    }

    void Collision(out Vector3 c, out Matrix4x4 R)
    {
        c = Vector3.zero;
        Matrix4x4 temp_A = Matrix4x4.zero;
        for (int i = 0; i < X.Length; i++)
        {
            for (int j = 0; j < planes.Length; j++)
            {
                Vector3 plane = planes[j];
                Vector3 normal = N[j];

                if (!TestCollision(X[i], plane, normal)) continue;
                X[i] -= Vector3.Dot(X[i] - plane, normal) * normal;

                if (Vector3.Dot(V[i], normal) > 0) continue;

                Vector3 vi_n = Vector3.Dot(V[i], normal) * normal;
                Vector3 vi_t = V[i] - vi_n;
                float a = Mathf.Max(1.0f - mu * (1.0f + restitution) * vi_n.magnitude / vi_t.magnitude, 0.0f);
                Vector3 vi_n_new = -restitution * vi_n;
                Vector3 vi_t_new = a * vi_t;
                V[i] = vi_n_new + vi_t_new;
            }

            c += X[i];
        }
        c /= X.Length;
        for (int i = 0; i < X.Length; i++)
        {
            temp_A[0, 0] += (X[i][0] - c[0]) * Q[i][0];
            temp_A[0, 1] += (X[i][0] - c[0]) * Q[i][1];
            temp_A[0, 2] += (X[i][0] - c[0]) * Q[i][2];
            temp_A[1, 0] += (X[i][1] - c[1]) * Q[i][0];
            temp_A[1, 1] += (X[i][1] - c[1]) * Q[i][1];
            temp_A[1, 2] += (X[i][1] - c[1]) * Q[i][2];
            temp_A[2, 0] += (X[i][2] - c[2]) * Q[i][0];
            temp_A[2, 1] += (X[i][2] - c[2]) * Q[i][1];
            temp_A[2, 2] += (X[i][2] - c[2]) * Q[i][2];
        }
        temp_A[3, 3] = 1;

        Matrix4x4 A = temp_A * QQt.inverse;
        R = Get_Rotation(A);
    }

    void InitEntitySpeed()
    {
        for (int i = 0; i < V.Length; i++)
        {
            V[i] = new Vector3(5, 2, 0);
        }
        restitution = 0.5f;
        launched = true;
    }

    void ResetEntityPos()
    {
        launched = false;
        Update_Mesh(reset_c, Matrix4x4.identity, 0);
    }

    // Update is called once per frame
    void Update()
    {
        dt = Time.deltaTime;

        //Game Control
        if (Input.GetKey("r"))
        {
            ResetEntityPos();
        }
        if (Input.GetKey("l") && !launched)
        {
            InitEntitySpeed();
        }

        if (launched != true) return;

        //Step 1: run a simple particle system.
        for (int i = 0; i < V.Length; i++)
        {
            V[i] *= linear_decay;
            V[i] += gravity * dt;  // mass is 1.0f
            X[i] += V[i] * dt;
        }

        // Step 2: Perform simple particle collision.
        // Step 3: Use shape matching to get new translation c and 
        // new rotation R.
        Vector3 c;
        Matrix4x4 R;
        Collision(out c, out R);
        // Update the mesh by c and R.
        Update_Mesh(c, R, 1 / dt);

    }
}
