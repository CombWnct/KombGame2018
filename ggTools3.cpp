#ifndef GGTOOLS3
#define GGTOOLS3

#include "ggTools3.hpp"

__int8 ggTools3::creater::SetScene(const aiScene* ARGscene, size_t ARGINDmesh,size_t maxAffected,size_t ARGINDmaterial,int ARGtexture) {

	this->targetScene = ARGscene;
	this->targetMesh = ARGscene->mMeshes[ARGINDmesh];

	//頂点　インデックス　頂点レベルボーン情報を確保する
	this->ARRvertexOriginal.resize(ARGscene->mMeshes[ARGINDmesh]->mNumVertices);
	this->ARRindexOriginal.resize(ARGscene->mMeshes[ARGINDmesh]->mNumFaces * 3);//面の三倍
	this->ARRboneInfoForVertex.resize(ARGscene->mMeshes[ARGINDmesh]->mNumVertices);//頂点の数

	//頂点レベルボーン情報を初期化しておく
	for (auto& i : this->ARRboneInfoForVertex)
		i.Init(maxAffected);

	//マテリアルの読み出し
	this->targetScene->mMaterials[ARGINDmaterial]->Get(AI_MATKEY_COLOR_DIFFUSE, this->diffuse);
	this->targetScene->mMaterials[ARGINDmaterial]->Get(AI_MATKEY_COLOR_SPECULAR, this->specular);
	this->GRtexture = ARGtexture;

	//頂点データをトレース
	for (const auto& i : this->ARRvertexOriginal | boost::adaptors::indexed()) {
		i.value().pos.x = this->targetMesh->mVertices[i.index()].x;
		i.value().pos.y = this->targetMesh->mVertices[i.index()].y;
		i.value().pos.z = this->targetMesh->mVertices[i.index()].z;

		i.value().norm.x = this->targetMesh->mNormals[i.index()].x;
		i.value().norm.y = this->targetMesh->mNormals[i.index()].y;
		i.value().norm.z = this->targetMesh->mNormals[i.index()].z;

		i.value().dif.r = this->diffuse.r * 255;
		i.value().dif.g = this->diffuse.g * 255;
		i.value().dif.b = this->diffuse.b * 255;
		i.value().dif.a = 255;

		i.value().spc.r = this->specular.r * 255;
		i.value().spc.g = this->specular.g * 255;
		i.value().spc.b = this->specular.b * 255;
		i.value().spc.a = 255;

		i.value().u = this->targetMesh->mTextureCoords[0][i.index()].x;
		i.value().v = this->targetMesh->mTextureCoords[0][i.index()].y;

	}

	//インデックスデータをトレース
	for (size_t i = 0; i < this->targetMesh->mNumFaces; i++)//i番目のフェイス
		for (size_t j = 0; j < this->targetMesh->mFaces[i].mNumIndices; j++)//j番目のインデックス
			this->ARRindexOriginal[i * 3 + j] = this->targetMesh->mFaces[i].mIndices[j];

	/*ここからボーン情報******************************************************/

	//ボーンレベルボーン情報を確保 ボーンの数
	this->BoneInfo.resize(this->targetMesh->mNumBones);
	this->NumBones = this->targetMesh->mNumBones;

	//ルートのインバースを取得
	this->GlobalInverseTransform = this->targetScene->mRootNode->mTransformation.Inverse();

	for (size_t i = 0; i < this->targetMesh->mNumBones; i++) {//i番目のボーン

		if (this->BoneMapping.find(this->targetMesh->mBones[i]->mName.C_Str()) == this->BoneMapping.end())//i番目のボーンの名前が登録されていなければ
			this->BoneMapping[this->targetMesh->mBones[i]->mName.C_Str()] = i;//ボーンのインデックスと名前を紐づけ
																			  //ボーンのオフセット行列を取得する
		this->BoneInfo[i].BoneOffset = this->targetMesh->mBones[i]->mOffsetMatrix;


		for (size_t j = 0; j < this->targetMesh->mBones[i]->mNumWeights; j++)//j番目のウェイト
			this->ARRboneInfoForVertex[this->targetMesh->mBones[i]->mWeights[j].mVertexId].AddBoneInfo(i, this->targetMesh->mBones[i]->mWeights[j].mWeight);
	}

	return true;
}
__int8 ggTools3::creater::GetReplicaMeshData(ggTools3::meshData* ret) {

	//まず返り値クリアする
	ret->ARRindexData.clear();
	ret->ARRvertexData.clear();

	//頂点データをトレース
	for (auto i : this->ARRvertexOriginal)
		ret->ARRvertexData.push_back(i);
	//インデックスデータをトレース
	for (auto i : this->ARRindexOriginal)
		ret->ARRindexData.push_back(i);

	ret->texture = this->GRtexture;

	return true;

}
__int8 ggTools3::creater::GetAnimationMeshData(ggTools3::meshData* ret, float timeInSecond) {
	//まずレプリカデータを取得
	this->GetReplicaMeshData(ret);
	//頂点ごとの変換行列とウェイト
	Matrix4f* ARRboneTransform;
	float* ARRweight;
	//変換行列の配列
	std::vector<Matrix4f> ARRtransform;
	BoneTransform(timeInSecond, ARRtransform);

	//ボーンに補正をかける
	for (Matrix4f& i : ARRtransform) {
		if (i.m[1][1] < 0)
			i.m[1][1] *= -1;
		if (i.m[2][2] < 0)
			i.m[2][2] *= -1;
		//しきい値以下の値は0として扱う
		for (size_t j = 0; j < 4; j++)
			for (size_t k = 0; k < 4; k++)
				if (fabs(i.m[j][k]) < 0.00001)
					i.m[j][k] = +0;
				//if (j == k)i.m[j][k] = 1;
				//else i.m[j][k] = +0;
	}


	//頂点ごとに変換していく
	for (size_t i = 0; i < (ret)->ARRvertexData.size(); i++) {

		//もしボーン情報が追加されていたら
		if (this->ARRboneInfoForVertex[i].enabledInfoSize > 0) {
			//頂点ごとの変換行列とウェイトを確保、トレースする
			ARRboneTransform = new Matrix4f[this->ARRboneInfoForVertex[i].enabledInfoSize];
			ARRweight = new float[this->ARRboneInfoForVertex[i].enabledInfoSize];
			for (size_t j = 0; j < this->ARRboneInfoForVertex[i].enabledInfoSize; j++) {//j番目のウェイト情報
				ARRboneTransform[j] = ARRtransform[this->ARRboneInfoForVertex[i].ARRboneID[j]];
				ARRweight[j] = this->ARRboneInfoForVertex[i].ARRweight[j];
			}

			//頂点にウェイトをかけたボーンを足し合わせていく
			static auto MultWeightedTransform = [](DxLib::VERTEX3D* ret,Matrix4f* ARRtransform,float* ARRweight,size_t ARGnum) {

				VERTEX3D ver;//合計を足し合わせていくためのキャッシュ
				ver.pos.x = 0;
				ver.pos.y = 0;
				ver.pos.z = 0;
				ver.norm.x = 0;
				ver.norm.y = 0;
				ver.norm.z = 0;
				VERTEX3D solve;//掛け合わされた頂点を保存するキャッシュ
				Matrix4f mat;//スカラー倍された行列を保存するキャッシュ

				//指定の行列をスカラー倍する
				static auto MultMatrixScalar = [](Matrix4f* mat, btScalar scalar, Matrix4f* ret) {
					for (size_t i = 0; i < 4; i++)
						for (size_t j = 0; j < 4; j++)
							ret->m[i][j] = mat->m[i][j] * scalar;
				};
				for (size_t i = 0; i < ARGnum; i++) {//i番目のウェイトと変換行列
					
					//変換行列をスカラー倍する
					MultMatrixScalar(ARRtransform + i, ARRweight[i], &mat);
					//頂点をスカラー倍した行列と掛け合わせる
					MultVertexMatrix(ret, &mat, &solve);
					//かけ合わせた頂点を足し合わせる
					ver.pos.x += solve.pos.x;
					ver.pos.y += solve.pos.y;
					ver.pos.z += solve.pos.z;
					ver.norm.x += solve.norm.x;
					ver.norm.y += solve.norm.y;
					ver.norm.z += solve.norm.z;
				}
				//返り値にトレースする
				ret->pos.x = ver.pos.x;
				ret->pos.y = ver.pos.y;
				ret->pos.z = ver.pos.z;
				ret->norm.x = ver.norm.x;
				ret->norm.y = ver.norm.y;
				ret->norm.z = ver.norm.z;

			};

			VERTEX3D a = ret->ARRvertexData[i];

			//実際足し合わせていく
			MultWeightedTransform(&(ret)->ARRvertexData[i], ARRboneTransform, ARRweight, this->ARRboneInfoForVertex[i].enabledInfoSize);

			if (a.pos.y != ret->ARRvertexData[i].pos.y)
				int aa = 0;

			//後処理
			delete[] ARRboneTransform;
			delete[] ARRweight;

		}

	}

	return true;
}
__int8 ggTools3::creater::CorrectAnimationMeshDataFailes(ggTools3::meshData* ARGmesh) {

	static Matrix4f mat;
	mat.InitRotateTransform(Quaternion(1, 0, 0, 0));

	ggTools3::MultEqualsVertexMatrix(&ARGmesh->ARRvertexData, &mat);

	return true;
}

__int8 ggTools3::DrawMeshData(ggTools3::meshData* ARGdata) {

	DrawPolygonIndexed3D(ARGdata->ARRvertexData.data(), ARGdata->ARRvertexData.size(), ARGdata->ARRindexData.data(), ARGdata->ARRindexData.size() / 3, ARGdata->texture, false);

	return true;
}
__int8 ggTools3::MultVertexMatrix(std::vector<DxLib::VERTEX3D>* vertexs, Matrix4f* matrix, std::vector<DxLib::VERTEX3D>* ret) {
	//初期化
	ret->resize(vertexs->size());
	//掛け合わせていく
	for (const auto &i : *vertexs | boost::adaptors::indexed()) {

		(*ret)[i.index()].pos.x = i.value().pos.x*matrix->m[0][0] + i.value().pos.y*matrix->m[0][1] + i.value().pos.z*matrix->m[0][2] + matrix->m[0][3];
		(*ret)[i.index()].pos.y = i.value().pos.x*matrix->m[1][0] + i.value().pos.y*matrix->m[1][1] + i.value().pos.z*matrix->m[1][2] + matrix->m[1][3];
		(*ret)[i.index()].pos.z = i.value().pos.x*matrix->m[2][0] + i.value().pos.y*matrix->m[2][1] + i.value().pos.z*matrix->m[2][2] + matrix->m[2][3];

		(*ret)[i.index()].norm.x = i.value().norm.x*matrix->m[0][0] + i.value().norm.y*matrix->m[0][1] + i.value().norm.z*matrix->m[0][2] + matrix->m[0][3];
		(*ret)[i.index()].norm.y = i.value().norm.x*matrix->m[1][0] + i.value().norm.y*matrix->m[1][1] + i.value().norm.z*matrix->m[1][2] + matrix->m[1][3];
		(*ret)[i.index()].norm.z = i.value().norm.x*matrix->m[2][0] + i.value().norm.y*matrix->m[2][1] + i.value().norm.z*matrix->m[2][2] + matrix->m[2][3];

	}

	return true;
}
__int8 ggTools3::MultVertexMatrix(DxLib::VERTEX3D* vertexs, Matrix4f* matrix,DxLib::VERTEX3D* ret) {

	float vw = 1, rw = 0;//座標のw要素
	float vwNorm = 1, rwNorm = 0;//法線のw要素

	ret->pos.x = vertexs->pos.x*matrix->m[0][0] + vertexs->pos.y*matrix->m[0][1] + vertexs->pos.z*matrix->m[0][2] + vw * matrix->m[0][3];
	ret->pos.y = vertexs->pos.x*matrix->m[1][0] + vertexs->pos.y*matrix->m[1][1] + vertexs->pos.z*matrix->m[1][2] + vw * matrix->m[1][3];
	ret->pos.z = vertexs->pos.x*matrix->m[2][0] + vertexs->pos.y*matrix->m[2][1] + vertexs->pos.z*matrix->m[2][2] + vw * matrix->m[2][3];
	rw =		 vertexs->pos.x*matrix->m[3][0] + vertexs->pos.y*matrix->m[3][1] + vertexs->pos.z*matrix->m[3][2] + vw * matrix->m[3][3];

	ret->norm.x = vertexs->norm.x*matrix->m[0][0] + vertexs->norm.y*matrix->m[0][1] + vertexs->norm.z*matrix->m[0][2] + vwNorm * matrix->m[0][3];
	ret->norm.y = vertexs->norm.x*matrix->m[1][0] + vertexs->norm.y*matrix->m[1][1] + vertexs->norm.z*matrix->m[1][2] + vwNorm * matrix->m[1][3];
	ret->norm.z = vertexs->norm.x*matrix->m[2][0] + vertexs->norm.y*matrix->m[2][1] + vertexs->norm.z*matrix->m[2][2] + vwNorm * matrix->m[2][3];
	rwNorm =	  vertexs->norm.x*matrix->m[3][0] + vertexs->norm.y*matrix->m[3][1] + vertexs->norm.z*matrix->m[3][2] + vwNorm * matrix->m[3][3];

	//wで割ってみる
	//ret->pos.x /= rw;
	//ret->pos.y /= rw;
	//ret->pos.z /= rw;
	//ret->norm.x /= rwNorm;
	//ret->norm.y /= rwNorm;
	//ret->norm.z /= rwNorm;
	return true;
}
__int8 ggTools3::MultEqualsVertexMatrix(std::vector<DxLib::VERTEX3D>* vertexs, Matrix4f* matrix) {

	std::vector<DxLib::VERTEX3D> solve;
	//掛け算を行う
	ggTools3::MultVertexMatrix(vertexs, matrix, &solve);
	//答えをトレース
	for (size_t i = 0; i < solve.size(); i++) {
		(*vertexs)[i].pos.x = solve[i].pos.x;
		(*vertexs)[i].pos.y = solve[i].pos.y;
		(*vertexs)[i].pos.z = solve[i].pos.z;

		(*vertexs)[i].norm.x = solve[i].norm.x;
		(*vertexs)[i].norm.y = solve[i].norm.y;
		(*vertexs)[i].norm.z = solve[i].norm.z;
	}



	return true;
}
__int8 ggTools3::ChangeBlenderCoordToDxCoord(ggTools3::meshData* ret) {

	static Matrix4f mat;
	mat.InitRotateTransform(Quaternion(0.707106781, 0, 0, -0.707106781));

	ggTools3::MultEqualsVertexMatrix(&ret->ARRvertexData, &mat);

	return true;
}

//秒単位でアニメーションの時間を与えてボーンごとの変換行列を受け取る
void ggTools3::creater::BoneTransform(float TimeInSeconds, vector<Matrix4f>& Transforms)
{
	Matrix4f Identity;
	Identity.InitIdentity();

	float TicksPerSecond = (float)(targetScene->mAnimations[0]->mTicksPerSecond != 0 ? targetScene->mAnimations[0]->mTicksPerSecond : 25.0f);
	float TimeInTicks = TimeInSeconds * TicksPerSecond;
	float AnimationTime = fmod(TimeInTicks, (float)targetScene->mAnimations[0]->mDuration);

	ReadNodeHeirarchy(AnimationTime, targetScene->mRootNode, Identity);

	Transforms.resize(this->NumBones);

	for (uint i = 0; i < NumBones; i++) {
		Transforms[i] = BoneInfo[i].FinalTransformation;
	}
}

//再帰的に全ボーンを検索して変換行列を回収する
void ggTools3::creater::ReadNodeHeirarchy(float AnimationTime, const aiNode* pNode, const Matrix4f& ParentTransform)
{
	string NodeName(pNode->mName.data);

	const aiAnimation* pAnimation = targetScene->mAnimations[0];

	Matrix4f NodeTransformation(pNode->mTransformation);

	const aiNodeAnim* pNodeAnim = FindNodeAnim(pAnimation, NodeName);

	if (pNodeAnim) {
		// Interpolate scaling and generate scaling transformation matrix
		aiVector3D Scaling;
		CalcInterpolatedScaling(Scaling, AnimationTime, pNodeAnim);
		Matrix4f ScalingM;
		ScalingM.InitScaleTransform(Scaling.x, Scaling.y, Scaling.z);

		// Interpolate rotation and generate rotation transformation matrix
		aiQuaternion RotationQ;
		CalcInterpolatedRotation(RotationQ, AnimationTime, pNodeAnim);
		Matrix4f RotationM = Matrix4f(RotationQ.GetMatrix());

		// Interpolate translation and generate translation transformation matrix
		aiVector3D Translation;
		CalcInterpolatedPosition(Translation, AnimationTime, pNodeAnim);
		Matrix4f TranslationM;
		TranslationM.InitTranslationTransform(Translation.x, Translation.y, Translation.z);

		// Combine the above transformations
		NodeTransformation = TranslationM * RotationM * ScalingM;
	}

	Matrix4f GlobalTransformation = ParentTransform * NodeTransformation;

	if (BoneMapping.find(NodeName) != BoneMapping.end()) {
		uint BoneIndex = BoneMapping[NodeName];
		BoneInfo[BoneIndex].FinalTransformation = GlobalInverseTransform * GlobalTransformation * BoneInfo[BoneIndex].BoneOffset;
	}

	for (uint i = 0; i < pNode->mNumChildren; i++) {
		ReadNodeHeirarchy(AnimationTime, pNode->mChildren[i], GlobalTransformation);
	}
}

//アニメーションの時間とノードから回転のクオータニオンを取得する
void ggTools3::creater::CalcInterpolatedRotation(aiQuaternion& Out, float AnimationTime, const aiNodeAnim* pNodeAnim)
{
	// we need at least two values to interpolate...
	if (pNodeAnim->mNumRotationKeys == 1) {
		Out = pNodeAnim->mRotationKeys[0].mValue;
		return;
	}

	uint RotationIndex = FindRotation(AnimationTime, pNodeAnim);
	uint NextRotationIndex = (RotationIndex + 1);
	assert(NextRotationIndex < pNodeAnim->mNumRotationKeys);
	float DeltaTime = (float)(pNodeAnim->mRotationKeys[NextRotationIndex].mTime - pNodeAnim->mRotationKeys[RotationIndex].mTime);
	float Factor = (AnimationTime - (float)pNodeAnim->mRotationKeys[RotationIndex].mTime) / DeltaTime;
	assert(Factor >= 0.0f && Factor <= 1.0f);
	const aiQuaternion& StartRotationQ = pNodeAnim->mRotationKeys[RotationIndex].mValue;
	const aiQuaternion& EndRotationQ = pNodeAnim->mRotationKeys[NextRotationIndex].mValue;
	aiQuaternion::Interpolate(Out, StartRotationQ, EndRotationQ, Factor);
	Out = Out.Normalize();
}
//アニメーションの時間とノードから移動のベクターを取得する
void ggTools3::creater::CalcInterpolatedPosition(aiVector3D& Out, float AnimationTime, const aiNodeAnim* pNodeAnim)
{
	if (pNodeAnim->mNumPositionKeys == 1) {
		Out = pNodeAnim->mPositionKeys[0].mValue;
		return;
	}

	uint PositionIndex = FindPosition(AnimationTime, pNodeAnim);
	uint NextPositionIndex = (PositionIndex + 1);
	assert(NextPositionIndex < pNodeAnim->mNumPositionKeys);
	float DeltaTime = (float)(pNodeAnim->mPositionKeys[NextPositionIndex].mTime - pNodeAnim->mPositionKeys[PositionIndex].mTime);
	float Factor = (AnimationTime - (float)pNodeAnim->mPositionKeys[PositionIndex].mTime) / DeltaTime;
	assert(Factor >= 0.0f && Factor <= 1.0f);
	const aiVector3D& Start = pNodeAnim->mPositionKeys[PositionIndex].mValue;
	const aiVector3D& End = pNodeAnim->mPositionKeys[NextPositionIndex].mValue;
	aiVector3D Delta = End - Start;
	Out = Start + Factor * Delta;
}
//アニメーションの時間とノードからスケールのベクターを取得する
void ggTools3::creater::CalcInterpolatedScaling(aiVector3D& Out, float AnimationTime, const aiNodeAnim* pNodeAnim)
{
	if (pNodeAnim->mNumScalingKeys == 1) {
		Out = pNodeAnim->mScalingKeys[0].mValue;
		return;
	}

	uint ScalingIndex = FindScaling(AnimationTime, pNodeAnim);
	uint NextScalingIndex = (ScalingIndex + 1);
	assert(NextScalingIndex < pNodeAnim->mNumScalingKeys);
	float DeltaTime = (float)(pNodeAnim->mScalingKeys[NextScalingIndex].mTime - pNodeAnim->mScalingKeys[ScalingIndex].mTime);
	float Factor = (AnimationTime - (float)pNodeAnim->mScalingKeys[ScalingIndex].mTime) / DeltaTime;
	assert(Factor >= 0.0f && Factor <= 1.0f);
	const aiVector3D& Start = pNodeAnim->mScalingKeys[ScalingIndex].mValue;
	const aiVector3D& End = pNodeAnim->mScalingKeys[NextScalingIndex].mValue;
	aiVector3D Delta = End - Start;
	Out = Start + Factor * Delta;
}
size_t ggTools3::creater::FindRotation(float AnimationTime, const aiNodeAnim* pNodeAnim)
{
	assert(pNodeAnim->mNumRotationKeys > 0);

	for (uint i = 0; i < pNodeAnim->mNumRotationKeys - 1; i++) {
		if (AnimationTime < (float)pNodeAnim->mRotationKeys[i + 1].mTime) {
			return i;
		}
	}

	assert(0);

	return 0;
}
size_t ggTools3::creater::FindPosition(float AnimationTime, const aiNodeAnim* pNodeAnim)
{
	for (uint i = 0; i < pNodeAnim->mNumPositionKeys - 1; i++) {
		if (AnimationTime < (float)pNodeAnim->mPositionKeys[i + 1].mTime) {
			return i;
		}
	}

	assert(0);

	return 0;
}
size_t ggTools3::creater::FindScaling(float AnimationTime, const aiNodeAnim* pNodeAnim)
{
	assert(pNodeAnim->mNumScalingKeys > 0);

	for (uint i = 0; i < pNodeAnim->mNumScalingKeys - 1; i++) {
		if (AnimationTime < (float)pNodeAnim->mScalingKeys[i + 1].mTime) {
			return i;
		}
	}

	assert(0);

	return 0;
}

const aiNodeAnim* ggTools3::creater::FindNodeAnim(const aiAnimation* pAnimation, const string NodeName)
{
	for (uint i = 0; i < pAnimation->mNumChannels; i++) {
		const aiNodeAnim* pNodeAnim = pAnimation->mChannels[i];

		if (string(pNodeAnim->mNodeName.data) == NodeName) {
			return pNodeAnim;
		}
	}

	return NULL;
}

__int8 ggTools3::rigidMeshData::Setup(ggTools3::meshData* ARGmesh,btQuaternion qrot,btVector3 pos,btVector3 inertia,btScalar mass){

	//トライアングルリストに必要なデータを宣言して確保する
	this->ARRvertexNode = new btScalar[ARGmesh->ARRvertexData.size() * 3];
	this->ARRindexBase = new int[ARGmesh->ARRindexData.size()];
	size_t NUMvertex = ARGmesh->ARRvertexData.size();
	size_t NUMtriangle = ARGmesh->ARRindexData.size() / 3;
	size_t SIvertexData = sizeof(btScalar) * 3;
	size_t SIindexTriangle = sizeof(int) * 3;

	//頂点要素をトレースする
	for (size_t i = 0; i < NUMvertex; i++) {
		ARRvertexNode[i * 3 + 0] = ARGmesh->ARRvertexData[i].pos.x;
		ARRvertexNode[i * 3 + 1] = ARGmesh->ARRvertexData[i].pos.y;
		ARRvertexNode[i * 3 + 2] = ARGmesh->ARRvertexData[i].pos.z;
	}
	//インデックスデータをトレースする
	for (size_t i = 0; i < NUMtriangle * 3; i++)
		ARRindexBase[i] = ARGmesh->ARRindexData[i];

	//bullet用のトライアングルリストを作成
	this->ARRvertexIndex = new btTriangleIndexVertexArray(NUMtriangle, ARRindexBase, SIindexTriangle, NUMvertex, ARRvertexNode, SIvertexData);
	//形状を作成
	this->shape = new btGImpactMeshShape(this->ARRvertexIndex);
	//形状を適用
	this->shape->updateBound();
	//姿勢を作成
	this->motionState = new btDefaultMotionState(btTransform(qrot, pos));
	//慣性モーメントの計算
	this->shape->calculateLocalInertia(mass, inertia);
	//剛体を作成
	this->rigidBody = new btRigidBody(mass, this->motionState, this->shape, inertia);

	return true;
}
ggTools3::rigidMeshData::~rigidMeshData() {

	delete this->rigidBody;
	delete this->motionState;
	delete this->shape;
	delete this->ARRvertexIndex;

	delete[] this->ARRvertexNode;
	delete[] this->ARRindexBase;

}
__int8 ggTools3::rigidMeshData::UpdateMeshData(ggTools3::meshData* ARGmesh) {
	btScalar* gotARRvertexNode;//受け取れるはずの頂点ノード
	int* gotARRindexBase;//受け取れるはずのインデックスベース
	int SIvertexNode, SItriangleIndex;
	int NUMvertex, NUMtriangleIndex;
	PHY_ScalarType TYPvertexNode = PHY_ScalarType::PHY_FLOAT, TYPindexBase = PHY_ScalarType::PHY_INTEGER;//ノードの方たち

	//頂点情報を収めるべきポインタを取得
	this->ARRvertexIndex->getLockedVertexIndexBase((unsigned char**)(&gotARRvertexNode), NUMvertex, TYPvertexNode, SIvertexNode, (unsigned char**)(&gotARRindexBase), SItriangleIndex, NUMtriangleIndex, TYPindexBase, 0);

	//アンロックする
	this->ARRvertexIndex->unLockVertexBase(0);

	//トレースしてね
	for (size_t i = 0; i < ARGmesh->ARRvertexData.size(); i++) {
		
		gotARRvertexNode[i * 3 + 0] = ARGmesh->ARRvertexData[i].pos.x;
		gotARRvertexNode[i * 3 + 1] = ARGmesh->ARRvertexData[i].pos.y;
		gotARRvertexNode[i * 3 + 2] = ARGmesh->ARRvertexData[i].pos.z;
	}
	for (size_t i = 0; i < ARGmesh->ARRindexData.size(); i++)
		gotARRindexBase[i] = ARGmesh->ARRindexData[i];

	//アンロックする
	this->ARRvertexIndex->unLockVertexBase(0);

	//アップデートする
	this->shape->postUpdate();

	return true;

}
btRigidBody* ggTools3::rigidMeshData::GetRigidBodyPointer() {
	return this->rigidBody;
}

__int8 ggTools3::GetSceneCreater(const aiScene* ARGscene, std::vector<ggTools3::creater>* ARGret,size_t maxAffected,std::vector<int>* const ARGARRgr) {

	bool enableGR = true;//画像が有効

	//返り値を確保
	ARGret->resize(ARGscene->mNumMeshes);
	//もし画像が足りていなければ
	if (!ARGARRgr)enableGR = false;
	else if (ARGscene->mNumMeshes != ARGARRgr->size())
		enableGR = false;

	//早速シーンをセットする
	for (size_t i = 0; i < ARGret->size(); i++)
		ARGret->at(i).SetScene(ARGscene, i, maxAffected, ARGscene->mMeshes[i]->mMaterialIndex, enableGR ? ARGARRgr->at(i) : DX_NONE_GRAPH);


	return true;
}
__int8 ggTools3::JoinTwoMeshData(meshData* const first, meshData* const second, meshData* ret) {
	//返り値をクリア
	ret->ARRindexData.clear();
	ret->ARRvertexData.clear();
	ret->texture = first->texture;

	//とりあえず頂点情報を結合
	for (size_t i = 0; i < first->ARRvertexData.size(); i++)ret->ARRvertexData.push_back(first->ARRvertexData[i]);
	for (size_t i = 0; i < second->ARRvertexData.size(); i++)ret->ARRvertexData.push_back(second->ARRvertexData[i]);

	//インデックス情報を結合
	for (size_t i = 0; i < first->ARRindexData.size(); i++)ret->ARRindexData.push_back(first->ARRindexData[i]);
	//二個目のデータはfirstのsize+値である
	for (size_t i = 0; i < second->ARRindexData.size(); i++)ret->ARRindexData.push_back(second->ARRindexData[i] + first->ARRvertexData.size());

	return true;

}
__int8 ggTools3::JoinAllMeshData(meshData* const ARRmeshData, size_t num, meshData* ret) {
	//キャッシュと返り値をクリア
	meshData cache;
	ret->ARRindexData.clear();
	cache.ARRindexData.clear();
	ret->ARRvertexData.clear();
	cache.ARRvertexData.clear();

	//足し合わせていきます
	for (size_t i = 0; i < num; i++) {
		//返り値とi番目のメッシュデータを結合しcacheに受け取る
		JoinTwoMeshData(ret, ARRmeshData + i, &cache);
		//返り値に結果をトレースする
		(*ret) = cache;
	}

	return true;
}


#endif