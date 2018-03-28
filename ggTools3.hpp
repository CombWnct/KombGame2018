#ifndef DX_GAME_FOR_SPRING_GENERAL_GGTOOLS3_GGTOOLS3_HPP
#define DX_GAME_FOR_SPRING_GENERAL_GGTOOLS3_GGTOOLS3_HPP

#include <DxLib.h>
#include <string>
#include <vector>
#include <map>

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#pragma comment (lib, "LinearMath.lib")
#pragma comment (lib, "BulletCollision.lib")
#pragma comment (lib, "BulletDynamics.lib")

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#pragma comment (lib,"assimp-vc140-mt.lib")

#include <ogldev_math_3d.h>

#include <boost/range/adaptor/indexed.hpp>
#include <boost/optional.hpp>
#include <boost/array.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
//#include <boost/filesystem.hpp>

namespace ggTools3 {

	//�`��⍄�̍\�z�ɕK�v�ȏ��
	struct meshData{

		std::vector<DxLib::VERTEX3D> ARRvertexData;//���_�f�[�^
		std::vector<unsigned short> ARRindexData;//�C���f�b�N�X�f�[�^

		int texture;//�e�N�X�`��

	};
	//���_���x���̃{�[�����
	struct boneInfoForVertex {

		size_t maxSize;//���̃{�[�����ɒǉ��ł���ID�̏��
		size_t enabledInfoSize;//�L���ȃ{�[�����̐�
		size_t* ARRboneID;//�{�[��id
		float* ARRweight;//����C���f�b�N�X�̃{�[��ID�̃E�F�C�g

							//��̒��_���󂯂���ő�̃{�[���̖{���������ɂ���
		__int8 Init(size_t _maxSize) {

			this->maxSize = _maxSize;
			this->enabledInfoSize = 0;
			this->ARRboneID = new size_t[maxSize];
			this->ARRweight = new float[maxSize];

			for (size_t i = 0; i < maxSize; i++) {
				this->ARRboneID[i] = 0;
				this->ARRweight[i] = 0;
			}

			return true;
		};
		//�K��̃R���X�g���N�^
		boneInfoForVertex() {
			return;
		}
		//�m�ۂ����������̌�n��������
		~boneInfoForVertex() {
			delete[] this->ARRboneID;
			delete[] this->ARRweight;
		}

		//�{�[������ǉ�����
		__int8 AddBoneInfo(size_t addBoneID, float addWeight) {

			if (maxSize == enabledInfoSize)return false;

			this->ARRboneID[this->enabledInfoSize] = addBoneID;
			this->ARRweight[this->enabledInfoSize] = addWeight;

			enabledInfoSize++;

			return true;
		}

	};
	//�{�[�����Ƃ̕ϊ��s����܂ރ{�[�����x���̃{�[�����
	struct BoneInfo
	{
		Matrix4f BoneOffset;
		Matrix4f FinalTransformation;

		BoneInfo()
		{
			BoneOffset.SetZero();
			FinalTransformation.SetZero();
		}
	};

	__int8 DrawMeshData(meshData* const mesh);

	//�w��̍s��𒸓_�Ɋ|�����킹��
	__int8 MultVertexMatrix(std::vector<DxLib::VERTEX3D>* vertexs, Matrix4f* matrix,std::vector<DxLib::VERTEX3D>* ret);
	__int8 MultVertexMatrix(DxLib::VERTEX3D* vertexs, Matrix4f* matrix, DxLib::VERTEX3D* ret);
	__int8 MultEqualsVertexMatrix(std::vector<DxLib::VERTEX3D>* vertexs, Matrix4f* matrix);

	//Blender�ō쐬�������b�V���f�[�^��DxLib�p�ɕϊ�����
	__int8 ChangeBlenderCoordToDxCoord(ggTools3::meshData* ret);

	//���b�V���f�[�^���쐬����
	class creater {

		//�I���W�i�����f���f�[�^
		const aiScene* targetScene;
		aiMesh* targetMesh;

		std::vector<DxLib::VERTEX3D> ARRvertexOriginal;//���f���̃I���W�i���̃f�[�^
		std::vector<unsigned short> ARRindexOriginal;//���f���̃I���W�i���f�[�^

		int GRtexture;// �e�N�X�`��

		aiColor3D diffuse;
		aiColor3D specular;

		//�{�[�����
		std::vector<boneInfoForVertex> ARRboneInfoForVertex;//���ׂĂ̒��_�ɑ΂��ĉe�������{�[����ID�ƃE�F�C�g�̔z����܂񂾔z��
		Matrix4f GlobalInverseTransform;//���[�g�m�[�h�̕ϊ��s��̃C���o�[�X�i�t�s��j
		std::map<std::string, size_t> BoneMapping; // �{�[�������L�[�Ƃ����{�[��ID�̃}�b�v
		std::vector<BoneInfo> BoneInfo;//�{�[�����i�{�[���̃I�t�Z�b�g�l���K�v�j
		size_t NumBones;//�����ɏ������@�\�����Ă��Ȃ�

		//�A�j���[�V�����n

		__int8 GetVertexMatrix(DxLib::VERTEX3D* const vertex, Matrix4f* matrix, float* weight, DxLib::VERTEX3D* ret, size_t NUMbone);
		//�ċA�I�ɑS�{�[�����������ĕϊ��s����������
		void ReadNodeHeirarchy(float AnimationTime, const aiNode* pNode, const Matrix4f& ParentTransform);
		//�A�j���[�V�����̎��Ԃƃm�[�h�����]�̃N�I�[�^�j�I�����擾����
		void CalcInterpolatedRotation(aiQuaternion& Out, float AnimationTime, const aiNodeAnim* pNodeAnim);
		//�A�j���[�V�����̎��Ԃƃm�[�h����ړ��̃x�N�^�[���擾����
		void CalcInterpolatedPosition(aiVector3D& Out, float AnimationTime, const aiNodeAnim* pNodeAnim);
		//�A�j���[�V�����̎��Ԃƃm�[�h����X�P�[���̃x�N�^�[���擾����
		void CalcInterpolatedScaling(aiVector3D& Out, float AnimationTime, const aiNodeAnim* pNodeAnim);
		//�A�j���[�V�����̎��Ԃƃm�[�h����ړ��̃x�N�^�[���擾����
		size_t FindRotation(float AnimationTime, const aiNodeAnim* pNodeAnim);
		size_t FindPosition(float AnimationTime, const aiNodeAnim* pNodeAnim);
		size_t FindScaling(float AnimationTime, const aiNodeAnim* pNodeAnim);
		const aiNodeAnim* FindNodeAnim(const aiAnimation* pAnimation, const string NodeName);
		//�b�P�ʂŃA�j���[�V�����̎��Ԃ�^���ă{�[�����Ƃ̕ϊ��s����󂯎��
		void BoneTransform(float TimeInSeconds, vector<Matrix4f>& Transforms);

	public:

		//�V�[����W�J
		__int8 SetScene(const aiScene* scene, size_t INDmesh,size_t maxAffected,size_t ARGINDmaterial,int ARGtexture);
		//�I���W�i���̃f�[�^���R�s�[�����̂�Ԃ�
		__int8 GetReplicaMeshData(meshData* ret);
		//�b�����w�肵�ăA�j���[�V�������s��
		__int8 GetAnimationMeshData(meshData* ret, float timeInSecond);
		//�A�j���[�V�������̍��W�n�̕s��v���C������
		__int8 CorrectAnimationMeshDataFailes(meshData* ARGmesh);


	};
	//���b�V���f�[�^��ǂݍ��ݍ��̉�����
	class rigidMeshData {

		btTriangleIndexVertexArray* ARRvertexIndex;
		btGImpactMeshShape* shape;
		btDefaultMotionState* motionState;
		btRigidBody* rigidBody;

		btScalar* ARRvertexNode;
		int* ARRindexBase;
	public:
		//���b�V����ݒ肷��
		__int8 Setup(ggTools3::meshData* ARGmesh, btQuaternion qrot, btVector3 pos, btVector3 inertia, btScalar mass);
		~rigidMeshData();
		//���b�V����ό`����
		__int8 UpdateMeshData(meshData* ARGmesh);
		//���̃I�u�W�F�N�g�̃|�C���^�𓾂�
		btRigidBody* GetRigidBodyPointer();
	};

	//�V�[����ǂݍ��݃��b�V���̐�����creator�̔z���Ԃ�
	__int8 GetSceneCreater(const aiScene* ARGscene, std::vector<ggTools3::creater>* ARGret, size_t maxAffected, std::vector<int>* const ARGARRgr);
	//���b�V���f�[�^2����������
	__int8 JoinTwoMeshData(meshData* const first, meshData* const second, meshData* ret);
	//�^����ꂽ���b�V���f�[�^�����ׂČ�������
	__int8 JoinAllMeshData(meshData* const ARRmeshData, size_t num, meshData* ret);



};

#endif 