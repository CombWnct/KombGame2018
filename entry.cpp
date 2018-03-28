#include "general.hpp"

namespace sample {

	constexpr unsigned __int16 winW = 640, winH = 480;

	char keyBuffer[256];
	btDiscreteDynamicsWorld* domain;//bulletライブラリ用のドメイン

	float screenX = 0;//スクリーン座標のx位置

	Assimp::Importer stageImporter;//モデルを読み込むもの
	const aiScene* originalData;//モデルのオリジナルデータ
	ggTools3::creater stageCreater;//モデルの剛体化に必要な情報を作成するもの
	ggTools3::meshData stageMeshData;//モデルの情報
	ggTools3::rigidMeshData stageRigid;//モデルの剛体
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

	//的関係
	std::vector<btRigidBody*> targets;
	constexpr int SIbulletMaxFar = 700;//玉が飛ぶ最大の距離
	constexpr int SIbombMaxFar = 700;//玉が飛ぶ最大の距離
	constexpr float bulletPower = 10000;
	constexpr float bulletSpeed = 100;

	//ブロック関係
	std::vector<btRigidBody*> blocks;

	std::vector<btRigidBody*> bullets;
	std::vector<btRigidBody*> bombs;//ボム

	//GOAL関係
	btRigidBody* rigidGoal;
	constexpr float goalX = 1380, goalY = 300;
};

int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	//ここはおまじないでよし
	general::InitDxLib();//dxlibの初期化
	general::InitBullet();//bulletの初期化
	//ここまで
	general::Init();
	general::CreateTarget();
	general::CreateBlocks();
	general::CreateGoal();
	//マウスカーソルを非表示
	SetMouseDispFlag(false);

	while (ProcessMessage() == 0 && ScreenFlip() == 0 && ClearDrawScreen() == 0 && sample::domain->stepSimulation(1.0 / 24.0, 32)&&GetHitKeyStateAll(sample::keyBuffer)==0) {

		btVector3 ball = sample::rigidChara->getCenterOfMassPosition();
		sample::rigidChara->setCenterOfMassTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(ball.x(), ball.y(), 0)));

		//画面のシフト
		if (ball.x() - sample::screenX > sample::winW / 2)sample::screenX += 2;
		//アクティベートする
		sample::rigidChara->activate();

		general::CheckKey();
		DrawGraph(0-sample::screenX, 0, sample::GRstage,true);
		DrawGraph(sample::winW - sample::screenX, 0, sample::GRstage2, true);

		general::DrawTarget();
		general::DrawBlocks();
		DrawGraph(ball.x() - sample::screenX - 10, sample::winH - ball.y() - 10, sample::GRchara, true);
		general::CheckBullets();//弾丸を設定
		general::CheckBombs();//ボムも
		if (general::CheckGoal())break;//ゴールしているかチェック　描画もここで行う
		//マウスカーソルを描画
		general::DrawCursor();
	}

	PlayMovie(sample::PAMOVending.c_str(), 1, DX_MOVIEPLAYTYPE_NORMAL);

	//GOALを処理しておく
	sample::domain->removeRigidBody(sample::rigidGoal);
	delete sample::rigidGoal->getMotionState();
	delete sample::rigidGoal->getCollisionShape();
	delete sample::rigidGoal;

	return 0;
}

__int8 general::InitDxLib() {

	//ウィンドウのスタイルを取得
	ChangeWindowMode(true);
	SetWindowStyleMode(2);

	//Dxlibの初期化
	if (DxLib::DxLib_Init() != 0)return false;

	SetDrawScreen(DX_SCREEN_BACK);

	return true;

}
__int8 general::Init() {
	
	//モデルの読み込み
	sample::originalData = sample::stageImporter.ReadFile(sample::PAstage, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_FlipUVs | aiProcess_FlipWindingOrder | aiProcess_MakeLeftHanded | 0);
	//モデル情報の作成機を作成
	sample::stageCreater.SetScene(sample::originalData, 0, 0, 0, NULL);
	//モデルの情報を抜き出す
	sample::stageCreater.GetReplicaMeshData(&sample::stageMeshData);
	//モデルに補正をかける
	ggTools3::ChangeBlenderCoordToDxCoord(&sample::stageMeshData);
	//モデルの剛体を作成 値はそれぞれ　モデル情報,姿勢,座標,慣性モーメント,質量
	sample::stageRigid.Setup(&sample::stageMeshData, btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0), btVector3(0, 0, 0), 0);
	//反射係数も設定しておきましょう
	sample::stageRigid.GetRigidBodyPointer()->setRestitution(0.2);

	//ドメインに追加してみましょう
	sample::domain->addRigidBody(sample::stageRigid.GetRigidBodyPointer());

	// 球体形状を設定
	btCollisionShape *sphere_shape = new btSphereShape(10);
	// 球体の初期位置・姿勢
	btQuaternion qrot(0, 0, 0, 1);
	btVector3 pos(0, 400, 0);
	btDefaultMotionState* motion_state = new btDefaultMotionState(btTransform(qrot, pos));
	// 慣性モーメントの計算
	btVector3 inertia(0, 0, 0);
	btScalar mass = 1.0;
	sphere_shape->calculateLocalInertia(mass, inertia);
	// 剛体オブジェクト生成(質量，位置姿勢，形状，慣性モーメントを設定)
	sample::rigidChara = new btRigidBody(mass, motion_state, sphere_shape, inertia);
	//反射係数
	sample::rigidChara->setRestitution(0.2);

	//ドメインに追加
	sample::domain->addRigidBody(sample::rigidChara);


	//画像も読み込み
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

	// 衝突検出方法の選択(デフォルトを選択)
	btDefaultCollisionConfiguration *config = new btDefaultCollisionConfiguration();
	btCollisionDispatcher *dispatcher = new btCollisionDispatcher(config);

	// ブロードフェーズ法の設定(Dynamic AABB tree method)
	btDbvtBroadphase *broadphase = new btDbvtBroadphase();

	// 拘束(剛体間リンク)のソルバ設定
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	// Bulletのワールド作成
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

	//弾丸を削除する
	for (const auto& i : sample::bullets | boost::adaptors::indexed()) {
		btVector3 pos = i.value()->getCenterOfMassPosition();
		btVector3 chara = sample::rigidChara->getCenterOfMassPosition();

		if (abs(pos.x() - chara.x()) > sample::SIbulletMaxFar || abs(pos.y() - chara.y()) > sample::SIbulletMaxFar) {
			btRigidBody* del = i.value();

			sample::bullets.erase(sample::bullets.begin() + i.index());
		}
	}

	//弾丸を生成する
	static bool pushed = false;
	if (!pushed&& (GetMouseInput()&MOUSE_INPUT_LEFT) != 0) {

		btVector3 PO3chara = sample::rigidChara->getCenterOfMassPosition();

		//マウスカーソルの位置を取得
		int mouseX, mouseY;
		GetMousePoint(&mouseX, &mouseY);
		//見かけの位置に変換する
		mouseX += sample::screenX;
		mouseY = sample::winH - mouseY;
		//方向ベクトルを生成
		VECTOR way = VScale(VNorm(VGet(-PO3chara.x() + mouseX, -PO3chara.y() + mouseY, 0)), sample::bulletPower);

		//姿勢を作成
		btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(PO3chara.x(), PO3chara.y()+30, PO3chara.z())));
		//形状を作成
		btCollisionShape *sphere_shape = new btSphereShape(5);
		// 慣性モーメントの計算
		btVector3 inertia(0, 0, 0);
		btScalar mass = 1.0;
		sphere_shape->calculateLocalInertia(mass, inertia);
		// 剛体オブジェクト生成(質量，位置姿勢，形状，慣性モーメントを設定)
		sample::bullets.push_back(new btRigidBody(mass, motionState, sphere_shape, inertia));
		//ドメインに追加
		sample::domain->addRigidBody(sample::bullets[sample::bullets.size() - 1]);
		//方向ベクトルを適応
		sample::bullets[sample::bullets.size() - 1]->applyCentralForce(btVector3(way.x, way.y, way.z));

	}
	//キー状態の更新
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

		//姿勢と形状を作成
		btCollisionShape* shape = new btBoxShape(btVector3(25, 25, 25));
		btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), i));
		//慣性モーメントを計算
		btVector3 inertia(0, 0, 0);
		shape->calculateLocalInertia(0.0,inertia);
		//剛体を追加
		sample::targets.push_back(new btRigidBody(0.0, motionState, shape, inertia));
		//ドメインにも
		sample::domain->addRigidBody(sample::targets[sample::targets.size() - 1]);

	}

	return true;
}
__int8 general::CreateBlocks() {
	std::vector<btVector3> poses = { btVector3(300,250,0),btVector3(500,250,0),btVector3(500,300,0),btVector3(300,300,0),btVector3(300,350,0),btVector3(500,350,0),btVector3(500,400,0),
	btVector3(900,300,0),btVector3(900,350,0) ,btVector3(900,400,0) ,btVector3(900,450,0) ,btVector3(1200,150,0),btVector3(1200,350,0) ,btVector3(1200,200,0) ,btVector3(1200,250,0),btVector3(1200,300,0),btVector3(1200,350,0) };

	for (auto i : poses) {

		//姿勢と形状を作成
		btCollisionShape* shape = new btBoxShape(btVector3(25, 25, 25));
		btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), i));
		//慣性モーメントを計算
		btVector3 inertia(0, 0, 0);
		shape->calculateLocalInertia(0.0, inertia);
		//剛体を追加
		sample::blocks.push_back(new btRigidBody(0.0, motionState, shape, inertia));
		//ドメインにも
		sample::domain->addRigidBody(sample::blocks[sample::blocks.size() - 1]);

	}
	return true;
}
__int8 general::DrawTarget() {

	for (auto i : sample::targets) {

		btVector3 pos = i->getCenterOfMassPosition();
		//的を補正
		i->setCenterOfMassTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(pos.x(), pos.y(), 0)));


		DrawGraph(pos.x() - 25 - sample::screenX, sample::winH - pos.y() - 25, sample::GRtarget, true);

		for (size_t i = 0; i < sample::domain->getDispatcher()->getNumManifolds(); i++) {

			//当たり判定情報を取得
			btPersistentManifold* manifold = sample::domain->getDispatcher()->getManifoldByIndexInternal(i);
			
			//衝突したオブジェクトを取得
			const btCollisionObject* obj1 = manifold->getBody0();
			const btCollisionObject* obj2 = manifold->getBody1();
			//アップキャストする
			const btRigidBody* rigid1 = btRigidBody::upcast(obj1);
			const btRigidBody* rigid2 = btRigidBody::upcast(obj2);

			//rigidが的もしくは弾か
			enum flagBulletOrTarget { other = 0, bullet = 1, target = -1 };
			flagBulletOrTarget FLrigid1 = flagBulletOrTarget::other, FLrigid2 = flagBulletOrTarget::other;
			//インデックスを保存
			size_t IND1, IND2;

			//弾の判定
			for (size_t j = 0; j < sample::bullets.size(); j++)
				if (sample::bullets[j] == rigid1) {//rigid1が弾ならば
					FLrigid1 = flagBulletOrTarget::bullet;
					IND1 = j;
				}
				else if (sample::bullets[j] == rigid2) {//rigid2が弾ならば
					FLrigid2 = flagBulletOrTarget::bullet;
					IND2 = j;
				}
			//的の判定
			for (size_t j = 0; j < sample::targets.size(); j++)
				if (sample::targets[j] == rigid1) {
					FLrigid1 = flagBulletOrTarget::target;
					IND1 = j;
				}
				else if (sample::targets[j] == rigid2) {
					FLrigid2 = flagBulletOrTarget::target;
					IND2 = j;
				}

			//あたったものが的と弾ならば
			if (FLrigid1 != FLrigid2 && FLrigid1 != 0 && FLrigid2 != 0){

				//ドメインから剛体を除去
				sample::domain->removeRigidBody(const_cast<btRigidBody*>(rigid1));
				sample::domain->removeRigidBody(const_cast<btRigidBody*>(rigid2));

				//配列からも削除する
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
		//的を補正
		i->setCenterOfMassTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(pos.x(), pos.y(), 0)));


		DrawGraph(pos.x() - 25 - sample::screenX, sample::winH - pos.y() - 25, sample::GRblock, true);

		for (size_t i = 0; i < sample::domain->getDispatcher()->getNumManifolds(); i++) {

			//当たり判定情報を取得
			btPersistentManifold* manifold = sample::domain->getDispatcher()->getManifoldByIndexInternal(i);

			//衝突したオブジェクトを取得
			const btCollisionObject* obj1 = manifold->getBody0();
			const btCollisionObject* obj2 = manifold->getBody1();
			//アップキャストする
			const btRigidBody* rigid1 = btRigidBody::upcast(obj1);
			const btRigidBody* rigid2 = btRigidBody::upcast(obj2);

			//rigidが的もしくは弾か
			enum flagbombOrblock { other = 0, bomb = 1, block = -1 };
			flagbombOrblock FLrigid1 = flagbombOrblock::other, FLrigid2 = flagbombOrblock::other;
			//インデックスを保存
			size_t IND1, IND2;

			//弾の判定
			for (size_t j = 0; j < sample::bombs.size(); j++)
				if (sample::bombs[j] == rigid1) {//rigid1が弾ならば
					FLrigid1 = flagbombOrblock::bomb;
					IND1 = j;
				}
				else if (sample::bombs[j] == rigid2) {//rigid2が弾ならば
					FLrigid2 = flagbombOrblock::bomb;
					IND2 = j;
				}
				//的の判定
				for (size_t j = 0; j < sample::blocks.size(); j++)
					if (sample::blocks[j] == rigid1) {
						FLrigid1 = flagbombOrblock::block;
						IND1 = j;
					}
					else if (sample::blocks[j] == rigid2) {
						FLrigid2 = flagbombOrblock::block;
						IND2 = j;
					}

					//あたったものが的と弾ならば
					if (FLrigid1 != FLrigid2 && FLrigid1 != 0 && FLrigid2 != 0) {

						//ドメインから剛体を除去
						sample::domain->removeRigidBody(const_cast<btRigidBody*>(rigid1));
						sample::domain->removeRigidBody(const_cast<btRigidBody*>(rigid2));

						//配列からも削除する
						(FLrigid1 == flagbombOrblock::bomb ? sample::bombs : sample::blocks).erase((FLrigid1 == flagbombOrblock::bomb ? sample::bombs : sample::blocks).begin() + IND1);
						(FLrigid2 == flagbombOrblock::bomb ? sample::bombs : sample::blocks).erase((FLrigid2 == flagbombOrblock::bomb ? sample::bombs : sample::blocks).begin() + IND2);

					}


		}
	}

	return true;
}
__int8 general::CheckBombs() {

	//ボムを削除する
	for (const auto& i : sample::bombs | boost::adaptors::indexed()) {
		btVector3 pos = i.value()->getCenterOfMassPosition();
		btVector3 chara = sample::rigidChara->getCenterOfMassPosition();

		if (abs(pos.x() - chara.x()) > sample::SIbombMaxFar || abs(pos.y() - chara.y()) > sample::SIbombMaxFar) {
			btRigidBody* del = i.value();

			sample::bombs.erase(sample::bombs.begin() + i.index());
		}
	}

	//ボムを生成する
	static bool pushed = false;
	if (!pushed&&(GetMouseInput()&MOUSE_INPUT_RIGHT)!=0) {

		btVector3 PO3chara = sample::rigidChara->getCenterOfMassPosition();

		//マウスカーソルの位置を取得
		int mouseX, mouseY;
		GetMousePoint(&mouseX, &mouseY);
		//見かけの位置に変換する
		mouseX += sample::screenX;
		mouseY = sample::winH - mouseY;
		//方向ベクトルを生成
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

			//向きが負なら
			if (mouseX - sample::rigidChara->getCenterOfMassPosition().x() < 0)
				way.x *= -1;
		}
		//方向ベクトルをpower倍
		way = VScale(VNorm(way),sample::bulletSpeed);

		//姿勢を作成
		btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(PO3chara.x(), PO3chara.y() + 30, PO3chara.z())));
		//形状を作成
		btCollisionShape *sphere_shape = new btSphereShape(5);
		// 慣性モーメントの計算
		btVector3 inertia(0, 0, 0);
		btScalar mass = 1.0;
		sphere_shape->calculateLocalInertia(mass, inertia);
		// 剛体オブジェクト生成(質量，位置姿勢，形状，慣性モーメントを設定)
		sample::bombs.push_back(new btRigidBody(mass, motionState, sphere_shape, inertia));
		//ドメインに追加
		sample::domain->addRigidBody(sample::bombs[sample::bombs.size() - 1]);
		//方向ベクトルを適応
		/*sample::bombs[sample::bombs.size() - 1]->applyCentralForce(btVector3(way.x, way.y, way.z));*/
		sample::bombs[sample::bombs.size() - 1]->setLinearVelocity(btVector3(way.x, way.y, way.z));

	}
	//キー状態の更新
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
	else exit(0);//解無し

	return true;
}
__int8 general::DrawCursor() {
	int x, y;
	GetMousePoint(&x, &y);
	DrawGraph(x - 25, y - 25, sample::GRcursor, true);

	return true;
}
__int8 general::CreateGoal() {

	//とりあえずGOALの形状を作成
	btCollisionShape* shape = new btBoxShape(btVector3(25, 25, 25));
	btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(sample::goalX, sample::goalY, 0)));
	btVector3 inertia(0, 0, 0);
	shape->calculateLocalInertia(0.0, inertia);
	sample::rigidGoal = new btRigidBody(0.0, motionState, shape, inertia);

	//ドメインに追加
	sample::domain->addRigidBody(sample::rigidGoal);

	return true;

}
__int8 general::CheckGoal() {

	//GOALの座標をゲット
	btVector3 PO3goal = sample::rigidGoal->getCenterOfMassPosition();
	//描画
	DrawGraph(PO3goal.x() - sample::screenX - 25, 480 - PO3goal.y() - 25, sample::GRgoal, true);

	//当たり判定
	for (size_t i = 0; i < sample::domain->getDispatcher()->getNumManifolds(); i++) {

		//当たり判定情報を取得
		btPersistentManifold* manifold = sample::domain->getDispatcher()->getManifoldByIndexInternal(i);

		//衝突したオブジェクトを取得
		const btCollisionObject* obj1 = manifold->getBody0();
		const btCollisionObject* obj2 = manifold->getBody1();
		//アップキャストする
		const btRigidBody* rigid1 = btRigidBody::upcast(obj1);
		const btRigidBody* rigid2 = btRigidBody::upcast(obj2);

		//rigidがキャラもしくはGOALか
		enum flagbombOrblock { other = 0, actor = 1, goal = -1 };
		flagbombOrblock FLrigid1 = flagbombOrblock::other, FLrigid2 = flagbombOrblock::other;
		//インデックスを保存
		size_t IND1, IND2;

		//弾の判定

		if (sample::rigidChara == rigid1) {//rigid1がアクターならば
			FLrigid1 = flagbombOrblock::actor;
		}
		else if (sample::rigidChara == rigid2) {//rigid2がアクターならば
			FLrigid2 = flagbombOrblock::actor;
		}

		//GOALの判定
		if (sample::rigidGoal == rigid1) {
			FLrigid1 = flagbombOrblock::goal;
		}
		else if (sample::rigidGoal == rigid2) {
			FLrigid2 = flagbombOrblock::goal;
		}

		//あたったものが的と弾ならば
		if (FLrigid1 != FLrigid2 && FLrigid1 != 0 && FLrigid2 != 0)return true;

	}
	return false;
}