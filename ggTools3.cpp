#ifndef GGTOOLS3
#define GGTOOLS3

#include "ggTools3.hpp"

__int8 ggTools3::creater::SetScene(const aiScene* ARGscene, size_t ARGINDmesh,size_t maxAffected,size_t ARGINDmaterial,int ARGtexture) {

	this->targetScene = ARGscene;
	this->targetMesh = ARGscene->mMeshes[ARGINDmesh];

	//���_�@�C���f�b�N�X�@���_���x���{�[�������m�ۂ���
	this->ARRvertexOriginal.resize(ARGscene->mMeshes[ARGINDmesh]->mNumVertices);
	this->ARRindexOriginal.resize(ARGscene->mMeshes[ARGINDmesh]->mNumFaces * 3);//�ʂ̎O�{
	this->ARRboneInfoForVertex.resize(ARGscene->mMeshes[ARGINDmesh]->mNumVertices);//���_�̐�

	//���_���x���{�[���������������Ă���
	for (auto& i : this->ARRboneInfoForVertex)
		i.Init(maxAffected);

	//�}�e���A���̓ǂݏo��
	this->targetScene->mMaterials[ARGINDmaterial]->Get(AI_MATKEY_COLOR_DIFFUSE, this->diffuse);
	this->targetScene->mMaterials[ARGINDmaterial]->Get(AI_MATKEY_COLOR_SPECULAR, this->specular);
	this->GRtexture = ARGtexture;

	//���_�f�[�^���g���[�X
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

	//�C���f�b�N�X�f�[�^���g���[�X
	for (size_t i = 0; i < this->targetMesh->mNumFaces; i++)//i�Ԗڂ̃t�F�C�X
		for (size_t j = 0; j < this->targetMesh->mFaces[i].mNumIndices; j++)//j�Ԗڂ̃C���f�b�N�X
			this->ARRindexOriginal[i * 3 + j] = this->targetMesh->mFaces[i].mIndices[j];

	/*��������{�[�����******************************************************/

	//�{�[�����x���{�[�������m�� �{�[���̐�
	this->BoneInfo.resize(this->targetMesh->mNumBones);
	this->NumBones = this->targetMesh->mNumBones;

	//���[�g�̃C���o�[�X���擾
	this->GlobalInverseTransform = this->targetScene->mRootNode->mTransformation.Inverse();

	for (size_t i = 0; i < this->targetMesh->mNumBones; i++) {//i�Ԗڂ̃{�[��

		if (this->BoneMapping.find(this->targetMesh->mBones[i]->mName.C_Str()) == this->BoneMapping.end())//i�Ԗڂ̃{�[���̖��O���o�^����Ă��Ȃ����
			this->BoneMapping[this->targetMesh->mBones[i]->mName.C_Str()] = i;//�{�[���̃C���f�b�N�X�Ɩ��O��R�Â�
																			  //�{�[���̃I�t�Z�b�g�s����擾����
		this->BoneInfo[i].BoneOffset = this->targetMesh->mBones[i]->mOffsetMatrix;


		for (size_t j = 0; j < this->targetMesh->mBones[i]->mNumWeights; j++)//j�Ԗڂ̃E�F�C�g
			this->ARRboneInfoForVertex[this->targetMesh->mBones[i]->mWeights[j].mVertexId].AddBoneInfo(i, this->targetMesh->mBones[i]->mWeights[j].mWeight);
	}

	return true;
}
__int8 ggTools3::creater::GetReplicaMeshData(ggTools3::meshData* ret) {

	//�܂��Ԃ�l�N���A����
	ret->ARRindexData.clear();
	ret->ARRvertexData.clear();

	//���_�f�[�^���g���[�X
	for (auto i : this->ARRvertexOriginal)
		ret->ARRvertexData.push_back(i);
	//�C���f�b�N�X�f�[�^���g���[�X
	for (auto i : this->ARRindexOriginal)
		ret->ARRindexData.push_back(i);

	ret->texture = this->GRtexture;

	return true;

}
__int8 ggTools3::creater::GetAnimationMeshData(ggTools3::meshData* ret, float timeInSecond) {
	//�܂����v���J�f�[�^���擾
	this->GetReplicaMeshData(ret);
	//���_���Ƃ̕ϊ��s��ƃE�F�C�g
	Matrix4f* ARRboneTransform;
	float* ARRweight;
	//�ϊ��s��̔z��
	std::vector<Matrix4f> ARRtransform;
	BoneTransform(timeInSecond, ARRtransform);

	//�{�[���ɕ␳��������
	for (Matrix4f& i : ARRtransform) {
		if (i.m[1][1] < 0)
			i.m[1][1] *= -1;
		if (i.m[2][2] < 0)
			i.m[2][2] *= -1;
		//�������l�ȉ��̒l��0�Ƃ��Ĉ���
		for (size_t j = 0; j < 4; j++)
			for (size_t k = 0; k < 4; k++)
				if (fabs(i.m[j][k]) < 0.00001)
					i.m[j][k] = +0;
				//if (j == k)i.m[j][k] = 1;
				//else i.m[j][k] = +0;
	}


	//���_���Ƃɕϊ����Ă���
	for (size_t i = 0; i < (ret)->ARRvertexData.size(); i++) {

		//�����{�[����񂪒ǉ�����Ă�����
		if (this->ARRboneInfoForVertex[i].enabledInfoSize > 0) {
			//���_���Ƃ̕ϊ��s��ƃE�F�C�g���m�ہA�g���[�X����
			ARRboneTransform = new Matrix4f[this->ARRboneInfoForVertex[i].enabledInfoSize];
			ARRweight = new float[this->ARRboneInfoForVertex[i].enabledInfoSize];
			for (size_t j = 0; j < this->ARRboneInfoForVertex[i].enabledInfoSize; j++) {//j�Ԗڂ̃E�F�C�g���
				ARRboneTransform[j] = ARRtransform[this->ARRboneInfoForVertex[i].ARRboneID[j]];
				ARRweight[j] = this->ARRboneInfoForVertex[i].ARRweight[j];
			}

			//���_�ɃE�F�C�g���������{�[���𑫂����킹�Ă���
			static auto MultWeightedTransform = [](DxLib::VERTEX3D* ret,Matrix4f* ARRtransform,float* ARRweight,size_t ARGnum) {

				VERTEX3D ver;//���v�𑫂����킹�Ă������߂̃L���b�V��
				ver.pos.x = 0;
				ver.pos.y = 0;
				ver.pos.z = 0;
				ver.norm.x = 0;
				ver.norm.y = 0;
				ver.norm.z = 0;
				VERTEX3D solve;//�|�����킳�ꂽ���_��ۑ�����L���b�V��
				Matrix4f mat;//�X�J���[�{���ꂽ�s���ۑ�����L���b�V��

				//�w��̍s����X�J���[�{����
				static auto MultMatrixScalar = [](Matrix4f* mat, btScalar scalar, Matrix4f* ret) {
					for (size_t i = 0; i < 4; i++)
						for (size_t j = 0; j < 4; j++)
							ret->m[i][j] = mat->m[i][j] * scalar;
				};
				for (size_t i = 0; i < ARGnum; i++) {//i�Ԗڂ̃E�F�C�g�ƕϊ��s��
					
					//�ϊ��s����X�J���[�{����
					MultMatrixScalar(ARRtransform + i, ARRweight[i], &mat);
					//���_���X�J���[�{�����s��Ɗ|�����킹��
					MultVertexMatrix(ret, &mat, &solve);
					//�������킹�����_�𑫂����킹��
					ver.pos.x += solve.pos.x;
					ver.pos.y += solve.pos.y;
					ver.pos.z += solve.pos.z;
					ver.norm.x += solve.norm.x;
					ver.norm.y += solve.norm.y;
					ver.norm.z += solve.norm.z;
				}
				//�Ԃ�l�Ƀg���[�X����
				ret->pos.x = ver.pos.x;
				ret->pos.y = ver.pos.y;
				ret->pos.z = ver.pos.z;
				ret->norm.x = ver.norm.x;
				ret->norm.y = ver.norm.y;
				ret->norm.z = ver.norm.z;

			};

			VERTEX3D a = ret->ARRvertexData[i];

			//���ۑ������킹�Ă���
			MultWeightedTransform(&(ret)->ARRvertexData[i], ARRboneTransform, ARRweight, this->ARRboneInfoForVertex[i].enabledInfoSize);

			if (a.pos.y != ret->ARRvertexData[i].pos.y)
				int aa = 0;

			//�㏈��
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
	//������
	ret->resize(vertexs->size());
	//�|�����킹�Ă���
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

	float vw = 1, rw = 0;//���W��w�v�f
	float vwNorm = 1, rwNorm = 0;//�@����w�v�f

	ret->pos.x = vertexs->pos.x*matrix->m[0][0] + vertexs->pos.y*matrix->m[0][1] + vertexs->pos.z*matrix->m[0][2] + vw * matrix->m[0][3];
	ret->pos.y = vertexs->pos.x*matrix->m[1][0] + vertexs->pos.y*matrix->m[1][1] + vertexs->pos.z*matrix->m[1][2] + vw * matrix->m[1][3];
	ret->pos.z = vertexs->pos.x*matrix->m[2][0] + vertexs->pos.y*matrix->m[2][1] + vertexs->pos.z*matrix->m[2][2] + vw * matrix->m[2][3];
	rw =		 vertexs->pos.x*matrix->m[3][0] + vertexs->pos.y*matrix->m[3][1] + vertexs->pos.z*matrix->m[3][2] + vw * matrix->m[3][3];

	ret->norm.x = vertexs->norm.x*matrix->m[0][0] + vertexs->norm.y*matrix->m[0][1] + vertexs->norm.z*matrix->m[0][2] + vwNorm * matrix->m[0][3];
	ret->norm.y = vertexs->norm.x*matrix->m[1][0] + vertexs->norm.y*matrix->m[1][1] + vertexs->norm.z*matrix->m[1][2] + vwNorm * matrix->m[1][3];
	ret->norm.z = vertexs->norm.x*matrix->m[2][0] + vertexs->norm.y*matrix->m[2][1] + vertexs->norm.z*matrix->m[2][2] + vwNorm * matrix->m[2][3];
	rwNorm =	  vertexs->norm.x*matrix->m[3][0] + vertexs->norm.y*matrix->m[3][1] + vertexs->norm.z*matrix->m[3][2] + vwNorm * matrix->m[3][3];

	//w�Ŋ����Ă݂�
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
	//�|���Z���s��
	ggTools3::MultVertexMatrix(vertexs, matrix, &solve);
	//�������g���[�X
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

//�b�P�ʂŃA�j���[�V�����̎��Ԃ�^���ă{�[�����Ƃ̕ϊ��s����󂯎��
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

//�ċA�I�ɑS�{�[�����������ĕϊ��s����������
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

//�A�j���[�V�����̎��Ԃƃm�[�h�����]�̃N�I�[�^�j�I�����擾����
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
//�A�j���[�V�����̎��Ԃƃm�[�h����ړ��̃x�N�^�[���擾����
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
//�A�j���[�V�����̎��Ԃƃm�[�h����X�P�[���̃x�N�^�[���擾����
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

	//�g���C�A���O�����X�g�ɕK�v�ȃf�[�^��錾���Ċm�ۂ���
	this->ARRvertexNode = new btScalar[ARGmesh->ARRvertexData.size() * 3];
	this->ARRindexBase = new int[ARGmesh->ARRindexData.size()];
	size_t NUMvertex = ARGmesh->ARRvertexData.size();
	size_t NUMtriangle = ARGmesh->ARRindexData.size() / 3;
	size_t SIvertexData = sizeof(btScalar) * 3;
	size_t SIindexTriangle = sizeof(int) * 3;

	//���_�v�f���g���[�X����
	for (size_t i = 0; i < NUMvertex; i++) {
		ARRvertexNode[i * 3 + 0] = ARGmesh->ARRvertexData[i].pos.x;
		ARRvertexNode[i * 3 + 1] = ARGmesh->ARRvertexData[i].pos.y;
		ARRvertexNode[i * 3 + 2] = ARGmesh->ARRvertexData[i].pos.z;
	}
	//�C���f�b�N�X�f�[�^���g���[�X����
	for (size_t i = 0; i < NUMtriangle * 3; i++)
		ARRindexBase[i] = ARGmesh->ARRindexData[i];

	//bullet�p�̃g���C�A���O�����X�g���쐬
	this->ARRvertexIndex = new btTriangleIndexVertexArray(NUMtriangle, ARRindexBase, SIindexTriangle, NUMvertex, ARRvertexNode, SIvertexData);
	//�`����쐬
	this->shape = new btGImpactMeshShape(this->ARRvertexIndex);
	//�`���K�p
	this->shape->updateBound();
	//�p�����쐬
	this->motionState = new btDefaultMotionState(btTransform(qrot, pos));
	//�������[�����g�̌v�Z
	this->shape->calculateLocalInertia(mass, inertia);
	//���̂��쐬
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
	btScalar* gotARRvertexNode;//�󂯎���͂��̒��_�m�[�h
	int* gotARRindexBase;//�󂯎���͂��̃C���f�b�N�X�x�[�X
	int SIvertexNode, SItriangleIndex;
	int NUMvertex, NUMtriangleIndex;
	PHY_ScalarType TYPvertexNode = PHY_ScalarType::PHY_FLOAT, TYPindexBase = PHY_ScalarType::PHY_INTEGER;//�m�[�h�̕�����

	//���_�������߂�ׂ��|�C���^���擾
	this->ARRvertexIndex->getLockedVertexIndexBase((unsigned char**)(&gotARRvertexNode), NUMvertex, TYPvertexNode, SIvertexNode, (unsigned char**)(&gotARRindexBase), SItriangleIndex, NUMtriangleIndex, TYPindexBase, 0);

	//�A�����b�N����
	this->ARRvertexIndex->unLockVertexBase(0);

	//�g���[�X���Ă�
	for (size_t i = 0; i < ARGmesh->ARRvertexData.size(); i++) {
		
		gotARRvertexNode[i * 3 + 0] = ARGmesh->ARRvertexData[i].pos.x;
		gotARRvertexNode[i * 3 + 1] = ARGmesh->ARRvertexData[i].pos.y;
		gotARRvertexNode[i * 3 + 2] = ARGmesh->ARRvertexData[i].pos.z;
	}
	for (size_t i = 0; i < ARGmesh->ARRindexData.size(); i++)
		gotARRindexBase[i] = ARGmesh->ARRindexData[i];

	//�A�����b�N����
	this->ARRvertexIndex->unLockVertexBase(0);

	//�A�b�v�f�[�g����
	this->shape->postUpdate();

	return true;

}
btRigidBody* ggTools3::rigidMeshData::GetRigidBodyPointer() {
	return this->rigidBody;
}

__int8 ggTools3::GetSceneCreater(const aiScene* ARGscene, std::vector<ggTools3::creater>* ARGret,size_t maxAffected,std::vector<int>* const ARGARRgr) {

	bool enableGR = true;//�摜���L��

	//�Ԃ�l���m��
	ARGret->resize(ARGscene->mNumMeshes);
	//�����摜������Ă��Ȃ����
	if (!ARGARRgr)enableGR = false;
	else if (ARGscene->mNumMeshes != ARGARRgr->size())
		enableGR = false;

	//�����V�[�����Z�b�g����
	for (size_t i = 0; i < ARGret->size(); i++)
		ARGret->at(i).SetScene(ARGscene, i, maxAffected, ARGscene->mMeshes[i]->mMaterialIndex, enableGR ? ARGARRgr->at(i) : DX_NONE_GRAPH);


	return true;
}
__int8 ggTools3::JoinTwoMeshData(meshData* const first, meshData* const second, meshData* ret) {
	//�Ԃ�l���N���A
	ret->ARRindexData.clear();
	ret->ARRvertexData.clear();
	ret->texture = first->texture;

	//�Ƃ肠�������_��������
	for (size_t i = 0; i < first->ARRvertexData.size(); i++)ret->ARRvertexData.push_back(first->ARRvertexData[i]);
	for (size_t i = 0; i < second->ARRvertexData.size(); i++)ret->ARRvertexData.push_back(second->ARRvertexData[i]);

	//�C���f�b�N�X��������
	for (size_t i = 0; i < first->ARRindexData.size(); i++)ret->ARRindexData.push_back(first->ARRindexData[i]);
	//��ڂ̃f�[�^��first��size+�l�ł���
	for (size_t i = 0; i < second->ARRindexData.size(); i++)ret->ARRindexData.push_back(second->ARRindexData[i] + first->ARRvertexData.size());

	return true;

}
__int8 ggTools3::JoinAllMeshData(meshData* const ARRmeshData, size_t num, meshData* ret) {
	//�L���b�V���ƕԂ�l���N���A
	meshData cache;
	ret->ARRindexData.clear();
	cache.ARRindexData.clear();
	ret->ARRvertexData.clear();
	cache.ARRvertexData.clear();

	//�������킹�Ă����܂�
	for (size_t i = 0; i < num; i++) {
		//�Ԃ�l��i�Ԗڂ̃��b�V���f�[�^��������cache�Ɏ󂯎��
		JoinTwoMeshData(ret, ARRmeshData + i, &cache);
		//�Ԃ�l�Ɍ��ʂ��g���[�X����
		(*ret) = cache;
	}

	return true;
}


#endif