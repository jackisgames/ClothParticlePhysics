using System;
using System.Collections;

namespace ClothSim.Physics
{
    public class ClothParticleSystem
    {
        private const int ConstraintResolveMax = 25;
        private readonly int m_numParticles;

        private readonly float[] m_x;
        private readonly float[] m_y;
        private readonly float[] m_z;

        private readonly float[] m_mass;
        private readonly bool[] m_kinematic;

        private readonly float[] m_px;
        private readonly float[] m_py;
        private readonly float[] m_pz;

        private readonly float[] m_fx;
        private readonly float[] m_fy;
        private readonly float[] m_fz;

        private readonly float m_gx;
        private readonly float m_gy;
        private readonly float m_gz;

        private readonly ConstraintData[] m_constraints;
        private readonly float m_resolveTarget;

        private readonly ICollisionObject[] m_collisions;

        public ClothParticleSystem(ParticleClothSettings settings)
        {
            m_numParticles = settings.Particles.Length;

            m_x = new float[m_numParticles];
            m_y = new float[m_numParticles];
            m_z = new float[m_numParticles];

            m_px = new float[m_numParticles];
            m_py = new float[m_numParticles];
            m_pz = new float[m_numParticles];

            m_fx = new float[m_numParticles];
            m_fy = new float[m_numParticles];
            m_fz = new float[m_numParticles];

            m_mass=new float[m_numParticles];
            m_kinematic=new bool[m_numParticles];

            settings.GetGravity(out m_gx, out m_gy, out m_gz);

            m_constraints = settings.Constraints;

            m_resolveTarget = settings.Constraints.Length * settings.ResolverThreshold;

            for (int i = 0; i < m_numParticles; ++i)
            {
                ParticleData p = settings.Particles[i];

                m_x[i] = m_px[i] = p.x;
                m_y[i] = m_py[i] = p.y;
                m_z[i] = m_pz[i] = p.z;
                m_mass[i] = p.Mass;
                m_kinematic[i] = p.IsKinematic;
            }

            m_collisions = settings.CollisionObjects;

            for (int i = 0; i < m_collisions.Length; ++i)
                m_collisions[i].Init(this);
        }

        private void UpdateForces(float dt)
        {
            for (int i = 0; i < m_numParticles; ++i)
            {
                float fx = m_fx[i];
                float fy = m_fy[i];
                float fz = m_fz[i];

                //todo apply forces
                fx = m_gx;
                fy = m_gy;
                fz = m_gz;

                m_fx[i] = fx;
                m_fy[i] = fy;
                m_fz[i] = fz;
            }
        }

        private void Verlet(float dt)
        {
            for (int i = 0; i < m_numParticles; ++i)
            {
                if(m_kinematic[i])
                    continue;

                float x = m_x[i];
                float y = m_y[i];
                float z = m_z[i];
                //temp
                float tx = x;
                float ty = y;
                float tz = z;

                
                //prev
                float px = m_px[i];
                float py = m_py[i];
                float pz = m_pz[i];

                //force + gravity
                float fx = m_fx[i];
                float fy = m_fy[i];
                float fz = m_fz[i];

                x += x - px + fx * dt * dt;
                y += y - py + fy * dt * dt;
                z += z - pz + fz * dt * dt;

                m_x[i] = x;
                m_y[i] = y;
                m_z[i] = z;

                m_px[i] = tx;
                m_py[i] = ty;
                m_pz[i] = tz;

            }
        }

        private void UpdateCollisions(float deltaTime)
        {
            for (int i = 0; i < m_numParticles; ++i)
            {
                for (int j = 0; j < m_collisions.Length; j++)
                {
                    m_collisions[j].CheckCollision(i);
                }
            }

            
        }

        private void UpdateConstraints(float dt)
        {
            float maxDiff = m_constraints.Length;
            int resolveCount = 0;
            while (maxDiff > m_resolveTarget)
            {

                maxDiff = 0;
                for (int i = 0; i < m_constraints.Length; ++i)
                {
                    ConstraintData constraint = m_constraints[i];

                    float x1 = m_x[constraint.ParticleAIndex];
                    float y1 = m_y[constraint.ParticleAIndex];
                    float z1 = m_z[constraint.ParticleAIndex];

                    float x2 = m_x[constraint.ParticleBIndex];
                    float y2 = m_y[constraint.ParticleBIndex];
                    float z2 = m_z[constraint.ParticleBIndex];

                    bool kinematic1 = m_kinematic[constraint.ParticleAIndex];
                    bool kinematic2 = m_kinematic[constraint.ParticleBIndex];

                    //both are kinematic ignore
                    if (kinematic1 && kinematic2)
                        continue;

                    float mass1 = m_mass[constraint.ParticleAIndex];
                    float mass2 = m_mass[constraint.ParticleBIndex];
                    float totalMass = mass1 + mass2;
                    mass1 /= totalMass;
                    mass2 /= totalMass;
                    if (kinematic1)
                    {
                        mass1 = 0;
                        mass2 = 1f;
                    }
                    else if (kinematic2)
                    {
                        mass1 = 1f;
                        mass2 = 0;
                    }
                    else
                    {
                        mass1 = 1f - mass1;
                        mass2 = 1f - mass2;
                    }



                    float dx = x2 - x1;
                    float dy = y2 - y1;
                    float dz = z2 - z1;

                    float deltaLength = (float) Math.Sqrt(dx * dx + dy * dy + dz * dz);
                    float diff = 0;
                    if (deltaLength >= constraint.Length)
                        diff = (deltaLength - constraint.Length) / (deltaLength);

                    dx *= diff;
                    dy *= diff;
                    dz *= diff;

                    

                    m_x[constraint.ParticleAIndex] = x1 + dx * mass1;
                    m_y[constraint.ParticleAIndex] = y1 + dy * mass1;
                    m_z[constraint.ParticleAIndex] = z1 + dz * mass1;

                    m_x[constraint.ParticleBIndex] = x2 - dx * mass2;
                    m_y[constraint.ParticleBIndex] = y2 - dy * mass2;
                    m_z[constraint.ParticleBIndex] = z2 - dz * mass2;

                    

                    maxDiff += diff;

                }
                
                resolveCount++;
                if (resolveCount == ConstraintResolveMax)
                {
                    break;
                }
            }
        }

        public void Step(float deltaTime)
        {
            UpdateConstraints(deltaTime);
            Verlet(deltaTime);
            UpdateForces(deltaTime);
            UpdateCollisions(deltaTime);

        }

        public bool IsKinematic(int index)
        {
            if (index >= m_numParticles)
            {
                return false;
            }
            return m_kinematic[index];
        }
        
        public void GetPosition(int index,out float x,out float y,out float z)
        {
            if (index >= m_numParticles)
            {
                x = y = z = 0;
                return;
            }
            x = m_x[index];
            y = m_y[index];
            z = m_z[index];
        }
        public void AddForce(int index, float x, float y, float z)
        {
            if (index >= m_numParticles)
            {
                return;
            }

            m_fx[index] = m_fx[index] + x;
            m_fy[index] = m_fy[index] + y;
            m_fz[index] = m_fz[index] + z;
        }

        public void SetForce(int index, float x, float y, float z)
        {
            if (index >= m_numParticles)
            {
                return;
            }

            m_fx[index] = x;
            m_fy[index] = y;
            m_fz[index] = z;
        }

        public void SetPosition(int index, float x, float y, float z,bool kinematicOnly)
        {
            if (index >= m_numParticles || kinematicOnly && !m_kinematic[index])
            {
                return;
            }

            m_x[index] = x;
            m_y[index] = y;
            m_z[index] = z;
        }
    }

    /// <summary>
    /// Contains initialization information for each particle
    /// </summary>
    public struct ParticleData
    {
        public float x;
        public float y;
        public float z;
        public float Mass;
        public bool IsKinematic;

        public ParticleData(float x, float y, float z, float mass,bool kinematic)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.Mass = mass;
            this.IsKinematic = kinematic;
        }
    }
    /// <summary>
    /// Contain constraint Data
    /// </summary>
    public struct ConstraintData
    {
        public int ParticleAIndex;
        public int ParticleBIndex;
        public float Length;
    }
    /// <summary>
    /// Initialization Settings for current cloth setup
    /// here you can set:
    /// 1. Gravity
    /// 2. Constraints
    /// 3. Particles Properties
    /// 4. particle constraints resolver quality
    /// 5. collisions
    /// </summary>
    public class ParticleClothSettings
    {
        private ParticleData[] m_particles;
        private ConstraintData[] m_constraints;
        private ICollisionObject[] m_collisionObjects;

        private float m_gx;
        private float m_gy;
        private float m_gz;

        private float m_resolverThreshold = .85f;

        public ParticleData[] Particles
        {
            get { return m_particles; }
        }

        public ConstraintData[] Constraints
        {
            get { return m_constraints; }
        }

        /// <summary>
        /// resolve threshold for constraint value between 0 and 1
        /// setting it to 0 or 1 might cause an issue
        /// </summary>
        public float ResolverThreshold
        {
            get { return m_resolverThreshold; }
            set { m_resolverThreshold = value; }
        }

        public ICollisionObject[] CollisionObjects
        {
            get { return m_collisionObjects; }
        }


        public void SetGravity(float x, float y, float z)
        {
            m_gx = x;
            m_gy = y;
            m_gz = z;
        }

        public void GetGravity(out float x, out float y, out float z)
        {
            x = m_gx;
            y = m_gy;
            z = m_gz;
        }

        public void SetCollisions(ICollisionObject[] collisions)
        {
            m_collisionObjects = collisions;
        }

        public void SetParticles(ParticleData[] particles,ConstraintData[] constraints)
        {
            m_particles = particles;
            m_constraints = constraints;

            SortByWeight comparer = new SortByWeight(m_particles);
            Array.Sort(m_constraints, comparer);
        }

        
    }

    class SortByWeight : IComparer
    {
        private ParticleData[] m_particles;

        internal SortByWeight(ParticleData[] particles)
        {
            m_particles = particles;
        }
        public int Compare(object x, object y)
        {
            ConstraintData a = (ConstraintData)x;
            ConstraintData b = (ConstraintData)y;

            ParticleData pa1 = m_particles[a.ParticleAIndex];
            ParticleData pa2 = m_particles[a.ParticleBIndex];

            ParticleData pb1 = m_particles[b.ParticleAIndex];
            ParticleData pb2 = m_particles[b.ParticleBIndex];

            if (pa1.IsKinematic || pa2.IsKinematic)
                return -1;
            if (pb1.IsKinematic || pb2.IsKinematic)
                return 1;

            if (Math.Min(pa1.Mass, pa2.Mass) < Math.Min(pb1.Mass, pb2.Mass))
                return -1;
            return 1;
        }
    }

    class SphereCollider:ICollisionObject
    {
        private float x;
        private float y;
        private float z;

        private float m_radius;
        private float m_radiusSqrt;

        private ClothParticleSystem m_particleSystem;

        public void Init(ClothParticleSystem particleSystem)
        {
            m_particleSystem = particleSystem;
        }

        public void CheckCollision(int index)
        {
            if (m_particleSystem.IsKinematic(index))
                return;

            float px, py, pz;
            m_particleSystem.GetPosition(index, out px, out py, out pz);

            float dx = px - x;
            float dy = py - y;
            float dz = pz - z;

            float distSqrt = dx * dx + dy * dy + dz * dz;
            if (distSqrt <= m_radiusSqrt)
            {
                float penetration = (m_radiusSqrt/distSqrt) * (m_radiusSqrt - distSqrt) / m_radiusSqrt;

                dx *= penetration;
                dy *= penetration;
                dz *= penetration;
                m_particleSystem.SetPosition(index, px + dx, py + dy, pz + dz, false);
                m_particleSystem.SetForce(index, 0, 0, 0);
            }
        }

        public void SetRadius(float radius)
        {
            m_radius = radius;
            m_radiusSqrt = m_radius * m_radius;
        }
        public void SetPosition(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }

    class PlaneCollider : ICollisionObject
    {
        private float x;
        private float y;
        private float z;

        private float m_nX;
        private float m_nY = 1;
        private float m_nZ;

        private ClothParticleSystem m_particleSystem;

        public void Init(ClothParticleSystem particleSystem)
        {
            m_particleSystem = particleSystem;
        }

        public void CheckCollision(int index)
        {
            if(m_particleSystem.IsKinematic(index))
                return;

            float px, py, pz;
            m_particleSystem.GetPosition(index, out px, out py, out pz);

            float dx = px - x;
            float dy = py - y;
            float dz = pz - z;

            float dot = m_nX * dx + m_nY * dy + m_nZ * dz;

            if (dot <= 0)
            {
                
                float fx = dot * m_nX;
                float fy = dot * m_nY;
                float fz = dot * m_nZ;

                px = px - fx;
                py = py - fy;
                pz = pz - fz;
                m_particleSystem.SetPosition(index, px, py, pz,false);
                m_particleSystem.SetForce(index, 0, 0, 0);

                
            }
        }

        public void SetPosition(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public void SetNormal(float x, float y, float z)
        {
            m_nX = x;
            m_nY = y;
            m_nZ = z;
        }
    }

    public interface ICollisionObject
    {
        void Init(ClothParticleSystem particleSystem);
        void CheckCollision(int index);
        void SetPosition(float x, float y, float z);
    }
}
