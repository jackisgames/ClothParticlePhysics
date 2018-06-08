using System;
using System.Collections;

namespace ClothSim.Physics
{
    public class ClothParticleSystem
    {
        private const float TerminalVelocity = 30;

        private readonly int m_numParticles;

        private readonly float[] m_x;
        private readonly float[] m_y;
        private readonly float[] m_z;

        private readonly float[] m_mass;
        private readonly float[] m_invmass;
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
            m_invmass = new float[m_numParticles];
            m_kinematic =new bool[m_numParticles];

            settings.GetGravity(out m_gx, out m_gy, out m_gz);

            m_constraints = settings.Constraints;

            for (int i = 0; i < m_numParticles; ++i)
            {
                ParticleData p = settings.Particles[i];

                m_x[i] = m_px[i] = p.x;
                m_y[i] = m_py[i] = p.y;
                m_z[i] = m_pz[i] = p.z;
                m_mass[i] = p.Mass;
                m_invmass[i] = 1f / p.Mass;
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

                float invMass = m_invmass[i];
                //todo apply forces
                fx = m_gx * invMass;
                fy = m_gy * invMass;
                fz = m_gz * invMass;

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
                //invmass
                float invMass = m_invmass[i];
                
                //prev
                float px = m_px[i];
                float py = m_py[i];
                float pz = m_pz[i];

                //force + gravity
                float fx = m_fx[i] * invMass;
                float fy = m_fy[i] * invMass;
                float fz = m_fz[i] * invMass;

                x += (x - px) + fx * dt * dt;
                y += (y - py) + fy * dt * dt;
                z += (z - pz) + fz * dt * dt;

                m_x[i] = x;
                m_y[i] = y;
                m_z[i] = z;

                m_px[i] = tx;
                m_py[i] = ty;
                m_pz[i] = tz;

            }
        }

        private float ClampVelocity(float v,float dt)
        {
            float tv = TerminalVelocity * dt;
            if (v > tv)
                return tv;
            if (v < -tv)
                return -tv;
            return v;

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

            for (int i = 0; i < m_constraints.Length; ++i)
            {
                ConstraintData constraint = m_constraints[i];


                bool kinematic1 = m_kinematic[constraint.ParticleAIndex];
                bool kinematic2 = m_kinematic[constraint.ParticleBIndex];

                //both are kinematic ignore
                if (kinematic1 && kinematic2)
                    continue;

                float x1 = m_x[constraint.ParticleAIndex];
                float y1 = m_y[constraint.ParticleAIndex];
                float z1 = m_z[constraint.ParticleAIndex];

                float x2 = m_x[constraint.ParticleBIndex];
                float y2 = m_y[constraint.ParticleBIndex];
                float z2 = m_z[constraint.ParticleBIndex];


                float dx = x2 - x1;
                float dy = y2 - y1;
                float dz = z2 - z1;


                float deltaLength = (float) Math.Sqrt(dx * dx + dy * dy + dz * dz);
                if (deltaLength <= 0)
                    continue;

                float invMass1 = kinematic1 ? 0 : m_invmass[constraint.ParticleAIndex];
                float invMass2 = kinematic2 ? 0 : m_invmass[constraint.ParticleBIndex];

                float diff = (deltaLength - constraint.Length) / (deltaLength*(invMass1+invMass2));
                dx *= diff;
                dy *= diff;
                dz *= diff;


                SetPosition(constraint.ParticleAIndex, x1 + dx * invMass1, y1 + dy * invMass1, z1 + dz * invMass1, false);
                SetPosition(constraint.ParticleBIndex, x2 - dx * invMass2, y2 - dy * invMass2, z2 - dz * invMass2, false);
            }

        }

        public void Step(float deltaTime)
        {
            
            Verlet(deltaTime);
            UpdateForces(deltaTime);
            UpdateCollisions(deltaTime);
            UpdateConstraints(deltaTime);

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
    /// 4. particle constraints resolver quality - deprecated
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

    class QuadCollider : ICollisionObject
    {
        public void Init(ClothParticleSystem particleSystem)
        {
            //throw new NotImplementedException();
        }

        public void CheckCollision(int index)
        {
            //throw new NotImplementedException();
        }

        public void SetPosition(float x, float y, float z)
        {
            //throw new NotImplementedException();
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
