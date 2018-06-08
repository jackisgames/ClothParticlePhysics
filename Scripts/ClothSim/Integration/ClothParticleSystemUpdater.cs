using System.Collections.Generic;
using ClothSim.Physics;
using UnityEngine;

namespace ClothSim.Integration
{
    //this is optional component to hook the system with unity
    public class ClothParticleSystemUpdater : MonoBehaviour
    {
        private ClothParticleSystem m_clothParticleSystem;
        private ClothParticleObject[] m_clothParticleObjects;

        [SerializeField]
        private Vector3 m_gravity=new Vector3(0,-10);

        private void Start()
        {
            m_clothParticleObjects = GetComponentsInChildren<ClothParticleObject>();

            ClothParticleConstraints[] clothParticleConstraints = GetComponentsInChildren<ClothParticleConstraints>();
            ParticleClothSettings particleSettings=new ParticleClothSettings();

            ParticleData[] particleDatas=new ParticleData[m_clothParticleObjects.Length];
            Stack<ConstraintData> constraintDatas = new Stack<ConstraintData>();

            for (int i = 0; i < m_clothParticleObjects.Length; i++)
            {
                ClothParticleObject obj = m_clothParticleObjects[i];
                obj.Index = i;
                particleDatas[i] = obj.ToParticleData();
            }

            for (int i = 0; i < clothParticleConstraints.Length; i++)
            {
                ClothParticleConstraints obj = clothParticleConstraints[i];

                int indexA = obj.GetComponent<ClothParticleObject>().Index;
                for (int j = 0; j < obj.OtherClothParticles.Length; j++)
                {
                    if (obj.ValidConnection(j))
                    {
                        ClothParticleObject other = obj.OtherClothParticles[j];
                        ConstraintData constraintData = new ConstraintData();
                        constraintData.ParticleAIndex = indexA;
                        constraintData.ParticleBIndex = other.Index;
                        constraintData.Length = Vector3.Distance(obj.transform.position, other.transform.position);
                        constraintDatas.Push(constraintData);
                    }
                }

            }
            particleSettings.SetGravity(m_gravity.x, m_gravity.y, m_gravity.z);
            particleSettings.SetParticles(particleDatas,constraintDatas.ToArray());

            //gather collision Objects
            ICollisionIntegrator[] collisions = GetComponentsInChildren<ICollisionIntegrator>();
            ICollisionObject[] collisionObjects = new ICollisionObject[collisions.Length];
            for (int i = 0; i < collisions.Length; i++)
            {
                collisionObjects[i] = collisions[i].CollisionObject();
            }
            particleSettings.SetCollisions(collisionObjects);

            m_clothParticleSystem = new ClothParticleSystem(particleSettings);
        }



        private void FixedUpdate()
        {
            for (int i = 0; i < m_clothParticleObjects.Length; ++i)
            {
                m_clothParticleSystem.SetPosition(i, m_clothParticleObjects[i].transform.position);
            }

            m_clothParticleSystem.Step(Time.fixedDeltaTime);

            for (int i = 0; i < m_clothParticleObjects.Length; ++i)
            {
                m_clothParticleObjects[i].transform.position = m_clothParticleSystem.GetPosition(i);
            }
        }
    }
}
