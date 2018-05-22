using UnityEngine;

namespace ClothSim.Integration
{
    [RequireComponent(typeof(ClothParticleObject))]
    public class ClothParticleConstraints:MonoBehaviour
    {
        [SerializeField]
        private ClothParticleObject[] m_otherClothParticles;

        public ClothParticleObject[] OtherClothParticles
        {
            get { return m_otherClothParticles; }
            set { m_otherClothParticles = value; }
        }

        public bool ValidConnection(int index)
        {
            if (index>=m_otherClothParticles.Length)
                return false;

            ClothParticleObject obj = m_otherClothParticles[index];
            if (obj == null)
                return false;

            ClothParticleObject thisClothParticleObject = GetComponent<ClothParticleObject>();
            //same object
            if (thisClothParticleObject.gameObject.GetInstanceID() == obj.gameObject.GetInstanceID())
                return false;
            //different parent
            if (thisClothParticleObject.transform.root.GetInstanceID() != obj.transform.root.GetInstanceID())
                return false;

            return true;
        }

        private void OnDrawGizmos()
        {
            if(m_otherClothParticles==null)
                return;

            for (int i = 0; i < m_otherClothParticles.Length; i++)
            {
                ClothParticleObject obj = m_otherClothParticles[i];
                if (obj != null)
                    Debug.DrawLine(transform.position, obj.transform.position, ValidConnection(i) ? Color.white : Color.gray);
            }
        }
    }
}
