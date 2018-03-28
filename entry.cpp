#include "general.hpp"

namespace sample {

	constexpr unsigned __int16 winW = 640, winH = 480;

	char keyBuffer[256];
	btDiscreteDynamicsWorld* domain;//bullet���C�u�����p�̃h���C��

	float screenX = 0;//�X�N���[�����W��x�ʒu

	Assimp::Importer stageImporter;//���f����ǂݍ��ނ���
	const aiScene* originalData;//���f���̃I���W�i���f�[�^
	ggTools3::creater stageCreater;//���f���̍��̉��ɕK�v�ȏ����쐬�������
	ggTools3::meshData stageMeshData;//���f���̏��
	ggTools3::rigidMeshData stageRigid;//���f���̍���
	const std::string PAstage = R"(stage.dae)";

	const std::string PAGRstage = R"(stage.png)";
	const std::string PAGRstage2 = R"(stage2.png)";

	const std::string PAGRchara = R"(graphs\chara.png)", PAGRbullet = R"(graphs\bullet.png)", PAGRtarget = R"(graphs\target.png)", PAGRbomb = R"(graphs\bomb.png)", PAGRblock = R"(graphs\block.png)", PAGRcursor = R"(graphs\cursor.png)", PAGRgoal = R"(graphs\goal.png)";
	int GRchara, GRbullet, GRtarget, GRbomb, GRblock, GRcursor, GRgoal;
	//movie
	const std::string PAMOVending = R"(movies\ending.mp4)";

	btRigidBody* rigidChara;

	int GRstage;
	int GRstage2;

	//�I�֌W
	std::vector<btRigidBody*> targets;
	constexpr int SIbulletMaxFar = 700;//�ʂ���ԍő�̋���
	constexpr int SIbombMaxFar = 700;//�ʂ���ԍő�̋���
	constexpr float bulletPower = 10000;
	constexpr float bulletSpeed = 100;

	//�u���b�N�֌W
	std::vector<btRigidBody*> blocks;

	std::vector<btRigidBody*> bullets;
	std::vector<btRigidBody*> bombs;//�{��

	//GOAL�֌W
	btRigidBody* rigidGoal;
	constexpr float goalX = 1380, goalY = 300;
};

int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	//�����͂��܂��Ȃ��ł悵
	general::InitDxLib();//dxlib�̏�����
	general::InitBullet();//bullet�̏�����
	//�����܂�
	general::Init();
	general::CreateTarget();
	general::CreateBlocks();
	general::CreateGoal();
	//�}�E�X�J�[�\�����\��
	SetMouseDispFlag(false);

	while (ProcessMessage() == 0 && ScreenFlip() == 0 && ClearDrawScreen() == 0 && sample::domain->stepSimulation(1.0 / 24.0, 32)&&GetHitKeyStateAll(sample::keyBuffer)==0) {

		btVector3 ball = sample::rigidChara->getCenterOfMassPosition();
		sample::rigidChara->setCenterOfMassTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(ball.x(), ball.y(), 0)));

		//��ʂ̃V�t�g
		if (ball.x() - sample::screenX > sample::winW / 2)sample::screenX += 2;
		//�A�N�e�B�x�[�g����
		sample::rigidChara->activate();

		general::CheckKey();
		DrawGraph(0-sample::screenX, 0, sample::GRstage,true);
		DrawGraph(sample::winW - sample::screenX, 0, sample::GRstage2, true);

		general::DrawTarget();
		general::DrawBlocks();
		DrawGraph(ball.x() - sample::screenX - 10, sample::winH - ball.y() - 10, sample::GRchara, true);
		general::CheckBullets();//�e�ۂ�ݒ�
		general::CheckBombs();//�{����
		if (general::CheckGoal())break;//�S�[�����Ă��邩�`�F�b�N�@�`��������ōs��
		//�}�E�X�J�[�\����`��
		general::DrawCursor();
	}

	PlayMovie(sample::PAMOVending.c_str(), 1, DX_MOVIEPLAYTYPE_NORMAL);

	//GOAL���������Ă���
	sample::domain->removeRigidBody(sample::rigidGoal);
	delete sample::rigidGoal->getMotionState();
	delete sample::rigidGoal->getCollisionShape();
	delete sample::rigidGoal;

	return 0;
}

__int8 general::InitDxLib() {

	//�E�B���h�E�̃X�^�C�����擾
	ChangeWindowMode(true);
	SetWindowStyleMode(2);

	//Dxlib�̏�����
	if (DxLib::DxLib_Init() != 0)return false;

	SetDrawScreen(DX_SCREEN_BACK);

	return true;

}
__int8 general::Init() {
	
	//���f���̓ǂݍ���
	sample::originalData = sample::stageImporter.ReadFile(sample::PAstage, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_FlipUVs | aiProcess_FlipWindingOrder | aiProcess_MakeLeftHanded | 0);
	//���f�����̍쐬�@���쐬
	sample::stageCreater.SetScene(sample::originalData, 0, 0, 0, NULL);
	//���f���̏��𔲂��o��
	sample::stageCreater.GetReplicaMeshData(&sample::stageMeshData);
	//���f���ɕ␳��������
	ggTools3::ChangeBlenderCoordToDxCoord(&sample::stageMeshData);
	//���f���̍��̂��쐬 �l�͂��ꂼ��@���f�����,�p��,���W,�������[�����g,����
	sample::stageRigid.Setup(&sample::stageMeshData, btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0), btVector3(0, 0, 0), 0);
	//���ˌW�����ݒ肵�Ă����܂��傤
	sample::stageRigid.GetRigidBodyPointer()->setRestitution(0.2);

	//�h���C���ɒǉ����Ă݂܂��傤
	sample::domain->addRigidBody(sample::stageRigid.GetRigidBodyPointer());

	// ���̌`���ݒ�
	btCollisionShape *sphere_shape = new btSphereShape(10);
	// ���̂̏����ʒu�E�p��
	btQuaternion qrot(0, 0, 0, 1);
	btVector3 pos(0, 400, 0);
	btDefaultMotionState* motion_state = new btDefaultMotionState(btTransform(qrot, pos));
	// �������[�����g�̌v�Z
	btVector3 inertia(0, 0, 0);
	btScalar mass = 1.0;
	sphere_shape->calculateLocalInertia(mass, inertia);
	// ���̃I�u�W�F�N�g����(���ʁC�ʒu�p���C�`��C�������[�����g��ݒ�)
	sample::rigidChara = new btRigidBody(mass, motion_state, sphere_shape, inertia);
	//���ˌW��
	sample::rigidChara->setRestitution(0.2);

	//�h���C���ɒǉ�
	sample::domain->addRigidBody(sample::rigidChara);


	//�摜���ǂݍ���
	sample::GRstage = LoadGraph(sample::PAGRstage.c_str());
	sample::GRstage2 = LoadGraph(sample::PAGRstage2.c_str());
	sample::GRchara = LoadGraph(sample::PAGRchara.c_str());
	sample::GRbullet = LoadGraph(sample::PAGRbullet.c_str());
	sample::GRtarget = LoadGraph(sample::PAGRtarget.c_str());
	sample::GRbomb = LoadGraph(sample::PAGRbomb.c_str());
	sample::GRblock = LoadGraph(sample::PAGRblock.c_str());
	sample::GRcursor = LoadGraph(sample::PAGRcursor.c_str());
	sample::GRgoal = LoadGraph(sample::PAGRgoal.c_str());
	return true;
}
__int8 general::InitBullet() {

	// �Փˌ��o���@�̑I��(�f�t�H���g��I��)
	btDefaultCollisionConfiguration *config = new btDefaultCollisionConfiguration();
	btCollisionDispatcher *dispatcher = new btCollisionDispatcher(config);

	// �u���[�h�t�F�[�Y�@�̐ݒ�(Dynamic AABB tree method)
	btDbvtBroadphase *broadphase = new btDbvtBroadphase();

	// �S��(���̊ԃ����N)�̃\���o�ݒ�
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	// Bullet�̃��[���h�쐬
	sample::domain = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config);
	sample::domain->setGravity(btVector3(0, -9.8, 0));

	return true;

}
__int8 general::CheckKey() {

	if (sample::keyBuffer[KEY_INPUT_A] == 1)
		sample::rigidChara->applyCentralForce(btVector3(-10, 0, 0));
	if (sample::keyBuffer[KEY_INPUT_D] == 1)
		sample::rigidChara->applyCentralForce(btVector3(10, 0, 0));
	if (sample::keyBuffer[KEY_INPUT_W] == 1)
		sample::rigidChara->applyCentralForce(btVector3(0, 40, 0));


	return true;
}
__int8 general::CheckBullets() {

	//�e�ۂ��폜����
	for (const auto& i : sample::bullets | boost::adaptors::indexed()) {
		btVector3 pos = i.value()->getCenterOfMassPosition();
		btVector3 chara = sample::rigidChara->getCenterOfMassPosition();

		if (abs(pos.x() - chara.x()) > sample::SIbulletMaxFar || abs(pos.y() - chara.y()) > sample::SIbulletMaxFar) {
			btRigidBody* del = i.value();

			sample::bullets.erase(sample::bullets.begin() + i.index());
		}
	}

	//�e�ۂ𐶐�����
	static bool pushed = false;
	if (!pushed&& (GetMouseInput()&MOUSE_INPUT_LEFT) != 0) {

		btVector3 PO3chara = sample::rigidChara->getCenterOfMassPosition();

		//�}�E�X�J�[�\���̈ʒu���擾
		int mouseX, mouseY;
		GetMousePoint(&mouseX, &mouseY);
		//�������̈ʒu�ɕϊ�����
		mouseX += sample::screenX;
		mouseY = sample::winH - mouseY;
		//�����x�N�g���𐶐�
		VECTOR way = VScale(VNorm(VGet(-PO3chara.x() + mouseX, -PO3chara.y() + mouseY, 0)), sample::bulletPower);

		//�p�����쐬
		btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(PO3chara.x(), PO3chara.y()+30, PO3chara.z())));
		//�`����쐬
		btCollisionShape *sphere_shape = new btSphereShape(5);
		// �������[�����g�̌v�Z
		btVector3 inertia(0, 0, 0);
		btScalar mass = 1.0;
		sphere_shape->calculateLocalInertia(mass, inertia);
		// ���̃I�u�W�F�N�g����(���ʁC�ʒu�p���C�`��C�������[�����g��ݒ�)
		sample::bullets.push_back(new btRigidBody(mass, motionState, sphere_shape, inertia));
		//�h���C���ɒǉ�
		sample::domain->addRigidBody(sample::bullets[sample::bullets.size() - 1]);
		//�����x�N�g����K��
		sample::bullets[sample::bullets.size() - 1]->applyCentralForce(btVector3(way.x, way.y, way.z));

	}
	//�L�[��Ԃ̍X�V
	if ((GetMouseInput()&MOUSE_INPUT_LEFT) != 0)pushed = true;
	else pushed = false;

	for (auto& i : sample::bullets) {
		btVector3 pos = i->getCenterOfMassPosition();
		//DrawCircle(pos.x() - sample::screenX,sample::winH- pos.y(), 5, GetColor(255, 255, 255));
		DrawGraph(pos.x() - sample::screenX - 5, sample::winH - pos.y() - 5, sample::GRbullet, true);
	}

	return true;
}
__int8 general::CreateTarget() {

	std::vector<btVector3> poses = { btVector3(600,300,0),btVector3(400,300,0),btVector3(700,300,0),btVector3(1200,300,0) };

	for (auto i : poses) {

		//�p���ƌ`����쐬
		btCollisionShape* shape = new btBoxShape(btVector3(25, 25, 25));
		btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), i));
		//�������[�����g���v�Z
		btVector3 inertia(0, 0, 0);
		shape->calculateLocalInertia(0.0,inertia);
		//���̂�ǉ�
		sample::targets.push_back(new btRigidBody(0.0, motionState, shape, inertia));
		//�h���C���ɂ�
		sample::domain->addRigidBody(sample::targets[sample::targets.size() - 1]);

	}

	return true;
}
__int8 general::CreateBlocks() {
	std::vector<btVector3> poses = { btVector3(300,250,0),btVector3(500,250,0),btVector3(500,300,0),btVector3(300,300,0),btVector3(300,350,0),btVector3(500,350,0),btVector3(500,400,0),
	btVector3(900,300,0),btVector3(900,350,0) ,btVector3(900,400,0) ,btVector3(900,450,0) ,btVector3(1200,150,0),btVector3(1200,350,0) ,btVector3(1200,200,0) ,btVector3(1200,250,0),btVector3(1200,300,0),btVector3(1200,350,0) };

	for (auto i : poses) {

		//�p���ƌ`����쐬
		btCollisionShape* shape = new btBoxShape(btVector3(25, 25, 25));
		btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), i));
		//�������[�����g���v�Z
		btVector3 inertia(0, 0, 0);
		shape->calculateLocalInertia(0.0, inertia);
		//���̂�ǉ�
		sample::blocks.push_back(new btRigidBody(0.0, motionState, shape, inertia));
		//�h���C���ɂ�
		sample::domain->addRigidBody(sample::blocks[sample::blocks.size() - 1]);

	}
	return true;
}
__int8 general::DrawTarget() {

	for (auto i : sample::targets) {

		btVector3 pos = i->getCenterOfMassPosition();
		//�I��␳
		i->setCenterOfMassTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(pos.x(), pos.y(), 0)));


		DrawGraph(pos.x() - 25 - sample::screenX, sample::winH - pos.y() - 25, sample::GRtarget, true);

		for (size_t i = 0; i < sample::domain->getDispatcher()->getNumManifolds(); i++) {

			//�����蔻������擾
			btPersistentManifold* manifold = sample::domain->getDispatcher()->getManifoldByIndexInternal(i);
			
			//�Փ˂����I�u�W�F�N�g���擾
			const btCollisionObject* obj1 = manifold->getBody0();
			const btCollisionObject* obj2 = manifold->getBody1();
			//�A�b�v�L���X�g����
			const btRigidBody* rigid1 = btRigidBody::upcast(obj1);
			const btRigidBody* rigid2 = btRigidBody::upcast(obj2);

			//rigid���I�������͒e��
			enum flagBulletOrTarget { other = 0, bullet = 1, target = -1 };
			flagBulletOrTarget FLrigid1 = flagBulletOrTarget::other, FLrigid2 = flagBulletOrTarget::other;
			//�C���f�b�N�X��ۑ�
			size_t IND1, IND2;

			//�e�̔���
			for (size_t j = 0; j < sample::bullets.size(); j++)
				if (sample::bullets[j] == rigid1) {//rigid1���e�Ȃ��
					FLrigid1 = flagBulletOrTarget::bullet;
					IND1 = j;
				}
				else if (sample::bullets[j] == rigid2) {//rigid2���e�Ȃ��
					FLrigid2 = flagBulletOrTarget::bullet;
					IND2 = j;
				}
			//�I�̔���
			for (size_t j = 0; j < sample::targets.size(); j++)
				if (sample::targets[j] == rigid1) {
					FLrigid1 = flagBulletOrTarget::target;
					IND1 = j;
				}
				else if (sample::targets[j] == rigid2) {
					FLrigid2 = flagBulletOrTarget::target;
					IND2 = j;
				}

			//�����������̂��I�ƒe�Ȃ��
			if (FLrigid1 != FLrigid2 && FLrigid1 != 0 && FLrigid2 != 0){

				//�h���C�����獄�̂�����
				sample::domain->removeRigidBody(const_cast<btRigidBody*>(rigid1));
				sample::domain->removeRigidBody(const_cast<btRigidBody*>(rigid2));

				//�z�񂩂���폜����
				(FLrigid1 == flagBulletOrTarget::bullet ? sample::bullets : sample::targets).erase((FLrigid1 == flagBulletOrTarget::bullet ? sample::bullets : sample::targets).begin() + IND1);
				(FLrigid2 == flagBulletOrTarget::bullet ? sample::bullets : sample::targets).erase((FLrigid2 == flagBulletOrTarget::bullet ? sample::bullets : sample::targets).begin() + IND2);

			}


		}
	}

	return true;
}
__int8 general::DrawBlocks() {

	for (auto i : sample::blocks) {

		btVector3 pos = i->getCenterOfMassPosition();
		//�I��␳
		i->setCenterOfMassTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(pos.x(), pos.y(), 0)));


		DrawGraph(pos.x() - 25 - sample::screenX, sample::winH - pos.y() - 25, sample::GRblock, true);

		for (size_t i = 0; i < sample::domain->getDispatcher()->getNumManifolds(); i++) {

			//�����蔻������擾
			btPersistentManifold* manifold = sample::domain->getDispatcher()->getManifoldByIndexInternal(i);

			//�Փ˂����I�u�W�F�N�g���擾
			const btCollisionObject* obj1 = manifold->getBody0();
			const btCollisionObject* obj2 = manifold->getBody1();
			//�A�b�v�L���X�g����
			const btRigidBody* rigid1 = btRigidBody::upcast(obj1);
			const btRigidBody* rigid2 = btRigidBody::upcast(obj2);

			//rigid���I�������͒e��
			enum flagbombOrblock { other = 0, bomb = 1, block = -1 };
			flagbombOrblock FLrigid1 = flagbombOrblock::other, FLrigid2 = flagbombOrblock::other;
			//�C���f�b�N�X��ۑ�
			size_t IND1, IND2;

			//�e�̔���
			for (size_t j = 0; j < sample::bombs.size(); j++)
				if (sample::bombs[j] == rigid1) {//rigid1���e�Ȃ��
					FLrigid1 = flagbombOrblock::bomb;
					IND1 = j;
				}
				else if (sample::bombs[j] == rigid2) {//rigid2���e�Ȃ��
					FLrigid2 = flagbombOrblock::bomb;
					IND2 = j;
				}
				//�I�̔���
				for (size_t j = 0; j < sample::blocks.size(); j++)
					if (sample::blocks[j] == rigid1) {
						FLrigid1 = flagbombOrblock::block;
						IND1 = j;
					}
					else if (sample::blocks[j] == rigid2) {
						FLrigid2 = flagbombOrblock::block;
						IND2 = j;
					}

					//�����������̂��I�ƒe�Ȃ��
					if (FLrigid1 != FLrigid2 && FLrigid1 != 0 && FLrigid2 != 0) {

						//�h���C�����獄�̂�����
						sample::domain->removeRigidBody(const_cast<btRigidBody*>(rigid1));
						sample::domain->removeRigidBody(const_cast<btRigidBody*>(rigid2));

						//�z�񂩂���폜����
						(FLrigid1 == flagbombOrblock::bomb ? sample::bombs : sample::blocks).erase((FLrigid1 == flagbombOrblock::bomb ? sample::bombs : sample::blocks).begin() + IND1);
						(FLrigid2 == flagbombOrblock::bomb ? sample::bombs : sample::blocks).erase((FLrigid2 == flagbombOrblock::bomb ? sample::bombs : sample::blocks).begin() + IND2);

					}


		}
	}

	return true;
}
__int8 general::CheckBombs() {

	//�{�����폜����
	for (const auto& i : sample::bombs | boost::adaptors::indexed()) {
		btVector3 pos = i.value()->getCenterOfMassPosition();
		btVector3 chara = sample::rigidChara->getCenterOfMassPosition();

		if (abs(pos.x() - chara.x()) > sample::SIbombMaxFar || abs(pos.y() - chara.y()) > sample::SIbombMaxFar) {
			btRigidBody* del = i.value();

			sample::bombs.erase(sample::bombs.begin() + i.index());
		}
	}

	//�{���𐶐�����
	static bool pushed = false;
	if (!pushed&&(GetMouseInput()&MOUSE_INPUT_RIGHT)!=0) {

		btVector3 PO3chara = sample::rigidChara->getCenterOfMassPosition();

		//�}�E�X�J�[�\���̈ʒu���擾
		int mouseX, mouseY;
		GetMousePoint(&mouseX, &mouseY);
		//�������̈ʒu�ɕϊ�����
		mouseX += sample::screenX;
		mouseY = sample::winH - mouseY;
		//�����x�N�g���𐶐�
		VECTOR way;
		{
			float ang;
			general::CalcBomb(abs(mouseX - sample::rigidChara->getCenterOfMassPosition().x()), mouseY - sample::rigidChara->getCenterOfMassPosition().y(), &ang);
			Matrix4f matQ; matQ.InitRotateTransform(Quaternion(0, 0, sin(ang / 2), cos(ang / 2)));
			auto mult = [](VECTOR *vertexs,Matrix4f *matrix,VECTOR* ret) {
				ret->x = vertexs->x*matrix->m[0][0] + vertexs->y*matrix->m[1][0] + vertexs->z*matrix->m[2][0] + matrix->m[3][0];
				ret->y = vertexs->x*matrix->m[0][1] + vertexs->y*matrix->m[1][1] + vertexs->z*matrix->m[2][1] + matrix->m[3][1];
				ret->z = vertexs->x*matrix->m[0][2] + vertexs->y*matrix->m[1][2] + vertexs->z*matrix->m[2][2] + matrix->m[3][2];
			};

			VECTOR gen = { 1, 0, 0 };
			mult(&gen, &matQ, &way);

			//���������Ȃ�
			if (mouseX - sample::rigidChara->getCenterOfMassPosition().x() < 0)
				way.x *= -1;
		}
		//�����x�N�g����power�{
		way = VScale(VNorm(way),sample::bulletSpeed);

		//�p�����쐬
		btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(PO3chara.x(), PO3chara.y() + 30, PO3chara.z())));
		//�`����쐬
		btCollisionShape *sphere_shape = new btSphereShape(5);
		// �������[�����g�̌v�Z
		btVector3 inertia(0, 0, 0);
		btScalar mass = 1.0;
		sphere_shape->calculateLocalInertia(mass, inertia);
		// ���̃I�u�W�F�N�g����(���ʁC�ʒu�p���C�`��C�������[�����g��ݒ�)
		sample::bombs.push_back(new btRigidBody(mass, motionState, sphere_shape, inertia));
		//�h���C���ɒǉ�
		sample::domain->addRigidBody(sample::bombs[sample::bombs.size() - 1]);
		//�����x�N�g����K��
		/*sample::bombs[sample::bombs.size() - 1]->applyCentralForce(btVector3(way.x, way.y, way.z));*/
		sample::bombs[sample::bombs.size() - 1]->setLinearVelocity(btVector3(way.x, way.y, way.z));

	}
	//�L�[��Ԃ̍X�V
	if ((GetMouseInput()&MOUSE_INPUT_RIGHT) != 0)pushed = true;
	else pushed = false;

	for (auto& i : sample::bombs) {
		btVector3 pos = i->getCenterOfMassPosition();
		//DrawCircle(pos.x() - sample::screenX,sample::winH- pos.y(), 5, GetColor(255, 255, 255));
		DrawGraph(pos.x() - sample::screenX - 15, sample::winH - pos.y() - 15, sample::GRbomb, true);
	}

	return true;
}
__int8 general::CalcBomb(int x,int y, float* ang) {


	float v = sample::bulletSpeed;

	float b = -1 * (2 * v*v*x) / (9.8*x*x);
	float c = 1 + (2 * v*v*y) / (9.8*x*x);
	float D = b * b - 4 * c;


	if (D >= 0) {

		float answer0 = atan((-b - sqrt(D)) / 2);
		float answer1 = atan((-b + sqrt(D)) / 2);

		float answer = answer0 > answer1 ? answer1 : answer0;

		*ang = answer;
	}
	else exit(0);//�𖳂�

	return true;
}
__int8 general::DrawCursor() {
	int x, y;
	GetMousePoint(&x, &y);
	DrawGraph(x - 25, y - 25, sample::GRcursor, true);

	return true;
}
__int8 general::CreateGoal() {

	//�Ƃ肠����GOAL�̌`����쐬
	btCollisionShape* shape = new btBoxShape(btVector3(25, 25, 25));
	btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(sample::goalX, sample::goalY, 0)));
	btVector3 inertia(0, 0, 0);
	shape->calculateLocalInertia(0.0, inertia);
	sample::rigidGoal = new btRigidBody(0.0, motionState, shape, inertia);

	//�h���C���ɒǉ�
	sample::domain->addRigidBody(sample::rigidGoal);

	return true;

}
__int8 general::CheckGoal() {

	//GOAL�̍��W���Q�b�g
	btVector3 PO3goal = sample::rigidGoal->getCenterOfMassPosition();
	//�`��
	DrawGraph(PO3goal.x() - sample::screenX - 25, 480 - PO3goal.y() - 25, sample::GRgoal, true);

	//�����蔻��
	for (size_t i = 0; i < sample::domain->getDispatcher()->getNumManifolds(); i++) {

		//�����蔻������擾
		btPersistentManifold* manifold = sample::domain->getDispatcher()->getManifoldByIndexInternal(i);

		//�Փ˂����I�u�W�F�N�g���擾
		const btCollisionObject* obj1 = manifold->getBody0();
		const btCollisionObject* obj2 = manifold->getBody1();
		//�A�b�v�L���X�g����
		const btRigidBody* rigid1 = btRigidBody::upcast(obj1);
		const btRigidBody* rigid2 = btRigidBody::upcast(obj2);

		//rigid���L������������GOAL��
		enum flagbombOrblock { other = 0, actor = 1, goal = -1 };
		flagbombOrblock FLrigid1 = flagbombOrblock::other, FLrigid2 = flagbombOrblock::other;
		//�C���f�b�N�X��ۑ�
		size_t IND1, IND2;

		//�e�̔���

		if (sample::rigidChara == rigid1) {//rigid1���A�N�^�[�Ȃ��
			FLrigid1 = flagbombOrblock::actor;
		}
		else if (sample::rigidChara == rigid2) {//rigid2���A�N�^�[�Ȃ��
			FLrigid2 = flagbombOrblock::actor;
		}

		//GOAL�̔���
		if (sample::rigidGoal == rigid1) {
			FLrigid1 = flagbombOrblock::goal;
		}
		else if (sample::rigidGoal == rigid2) {
			FLrigid2 = flagbombOrblock::goal;
		}

		//�����������̂��I�ƒe�Ȃ��
		if (FLrigid1 != FLrigid2 && FLrigid1 != 0 && FLrigid2 != 0)return true;

	}
	return false;
}