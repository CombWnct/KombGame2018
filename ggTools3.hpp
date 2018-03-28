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

	//描画や剛体構築に必要な情報
	struct meshData{

		std::vector<DxLib::VERTEX3D> ARRvertexData;//頂点データ
		std::vector<unsigned short> ARRindexData;//インデックスデータ

		int texture;//テクスチャ

	};
	//頂点レベルのボーン情報
	struct boneInfoForVertex {

		size_t maxSize;//このボーン情報に追加できるIDの上限
		size_t enabledInfoSize;//有効なボーン情報の数
		size_t* ARRboneID;//ボーンid
		float* ARRweight;//同一インデックスのボーンIDのウェイト

							//一つの頂点が受けうる最大のボーンの本数を引数にする
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
		//規定のコンストラクタ
		boneInfoForVertex() {
			return;
		}
		//確保したメモリの後始末をする
		~boneInfoForVertex() {
			delete[] this->ARRboneID;
			delete[] this->ARRweight;
		}

		//ボーン情報を追加する
		__int8 AddBoneInfo(size_t addBoneID, float addWeight) {

			if (maxSize == enabledInfoSize)return false;

			this->ARRboneID[this->enabledInfoSize] = addBoneID;
			this->ARRweight[this->enabledInfoSize] = addWeight;

			enabledInfoSize++;

			return true;
		}

	};
	//ボーンごとの変換行列を含むボーンレベルのボーン情報
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

	//指定の行列を頂点に掛け合わせる
	__int8 MultVertexMatrix(std::vector<DxLib::VERTEX3D>* vertexs, Matrix4f* matrix,std::vector<DxLib::VERTEX3D>* ret);
	__int8 MultVertexMatrix(DxLib::VERTEX3D* vertexs, Matrix4f* matrix, DxLib::VERTEX3D* ret);
	__int8 MultEqualsVertexMatrix(std::vector<DxLib::VERTEX3D>* vertexs, Matrix4f* matrix);

	//Blenderで作成したメッシュデータをDxLib用に変換する
	__int8 ChangeBlenderCoordToDxCoord(ggTools3::meshData* ret);

	//メッシュデータを作成する
	class creater {

		//オリジナルモデルデータ
		const aiScene* targetScene;
		aiMesh* targetMesh;

		std::vector<DxLib::VERTEX3D> ARRvertexOriginal;//モデルのオリジナルのデータ
		std::vector<unsigned short> ARRindexOriginal;//モデルのオリジナルデータ

		int GRtexture;// テクスチャ

		aiColor3D diffuse;
		aiColor3D specular;

		//ボーン情報
		std::vector<boneInfoForVertex> ARRboneInfoForVertex;//すべての頂点に対して影響されるボーンのIDとウェイトの配列を含んだ配列
		Matrix4f GlobalInverseTransform;//ルートノードの変換行列のインバース（逆行列）
		std::map<std::string, size_t> BoneMapping; // ボーン名をキーとしたボーンIDのマップ
		std::vector<BoneInfo> BoneInfo;//ボーン情報（ボーンのオフセット値が必要）
		size_t NumBones;//こいつに初期化機能がついていない

		//アニメーション系

		__int8 GetVertexMatrix(DxLib::VERTEX3D* const vertex, Matrix4f* matrix, float* weight, DxLib::VERTEX3D* ret, size_t NUMbone);
		//再帰的に全ボーンを検索して変換行列を回収する
		void ReadNodeHeirarchy(float AnimationTime, const aiNode* pNode, const Matrix4f& ParentTransform);
		//アニメーションの時間とノードから回転のクオータニオンを取得する
		void CalcInterpolatedRotation(aiQuaternion& Out, float AnimationTime, const aiNodeAnim* pNodeAnim);
		//アニメーションの時間とノードから移動のベクターを取得する
		void CalcInterpolatedPosition(aiVector3D& Out, float AnimationTime, const aiNodeAnim* pNodeAnim);
		//アニメーションの時間とノードからスケールのベクターを取得する
		void CalcInterpolatedScaling(aiVector3D& Out, float AnimationTime, const aiNodeAnim* pNodeAnim);
		//アニメーションの時間とノードから移動のベクターを取得する
		size_t FindRotation(float AnimationTime, const aiNodeAnim* pNodeAnim);
		size_t FindPosition(float AnimationTime, const aiNodeAnim* pNodeAnim);
		size_t FindScaling(float AnimationTime, const aiNodeAnim* pNodeAnim);
		const aiNodeAnim* FindNodeAnim(const aiAnimation* pAnimation, const string NodeName);
		//秒単位でアニメーションの時間を与えてボーンごとの変換行列を受け取る
		void BoneTransform(float TimeInSeconds, vector<Matrix4f>& Transforms);

	public:

		//シーンを展開
		__int8 SetScene(const aiScene* scene, size_t INDmesh,size_t maxAffected,size_t ARGINDmaterial,int ARGtexture);
		//オリジナルのデータをコピーたものを返す
		__int8 GetReplicaMeshData(meshData* ret);
		//秒数を指定してアニメーションを行う
		__int8 GetAnimationMeshData(meshData* ret, float timeInSecond);
		//アニメーション時の座標系の不一致を修正する
		__int8 CorrectAnimationMeshDataFailes(meshData* ARGmesh);


	};
	//メッシュデータを読み込み剛体化する
	class rigidMeshData {

		btTriangleIndexVertexArray* ARRvertexIndex;
		btGImpactMeshShape* shape;
		btDefaultMotionState* motionState;
		btRigidBody* rigidBody;

		btScalar* ARRvertexNode;
		int* ARRindexBase;
	public:
		//メッシュを設定する
		__int8 Setup(ggTools3::meshData* ARGmesh, btQuaternion qrot, btVector3 pos, btVector3 inertia, btScalar mass);
		~rigidMeshData();
		//メッシュを変形する
		__int8 UpdateMeshData(meshData* ARGmesh);
		//剛体オブジェクトのポインタを得る
		btRigidBody* GetRigidBodyPointer();
	};

	//シーンを読み込みメッシュの数だけcreatorの配列を返す
	__int8 GetSceneCreater(const aiScene* ARGscene, std::vector<ggTools3::creater>* ARGret, size_t maxAffected, std::vector<int>* const ARGARRgr);
	//メッシュデータ2つを結合する
	__int8 JoinTwoMeshData(meshData* const first, meshData* const second, meshData* ret);
	//与えられたメッシュデータをすべて結合する
	__int8 JoinAllMeshData(meshData* const ARRmeshData, size_t num, meshData* ret);



};

#endif 