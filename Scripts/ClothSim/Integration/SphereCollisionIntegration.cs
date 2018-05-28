using ClothSim.Physics;
using UnityEngine;

namespace ClothSim.Integration
{
    public class SphereCollisionIntegration : MonoBehaviour,ICollisionIntegrator
    {
        private readonly ClothSim.Physics.SphereCollider m_sphereCollider = new ClothSim.Physics.SphereCollider();

        [SerializeField,Range(0,1.5f)]
        private float m_radius;

        private void Update()
        {
            Vector3 position = transform.position;
            m_sphereCollider.SetPosition(position.x, position.y, position.z);
            m_sphereCollider.SetRadius(m_radius);
        }

        private void OnDrawGizmos()
        {
            Gizmos.color=Color.green;
            Gizmos.DrawSphere(transform.position, m_radius);
        }

        public ICollisionObject CollisionObject()
        {
            return m_sphereCollider;
        }
    }
}
