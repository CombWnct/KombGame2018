#ifndef KOMB_GAME_GENERAL_GENERAL_HPP_21_MAR
#define KOMB_GAME_GENERAL_GENERAL_HPP_21_MAR

#include "ggTools3.hpp"

int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int);
namespace general {

	__int8 LoadModelData(std::vector<ggTools3::meshData>* mesh, std::string PAMOD);//モデルを読み込む

	__int8 InitDxLib();//dxlibを初期化するよ
	__int8 InitBullet();//bulletを初期化するよ
	__int8 Init();//名前空間を初期化
	__int8 CheckKey();
	__int8 CheckBullets();//弾丸をチェック
	__int8 CreateTarget();//的を作成
	__int8 CreateBlocks();//ブロックを作成
	__int8 DrawTarget();
	__int8 DrawBlocks();
	__int8 CheckBombs();
	__int8 CalcBomb(int targetX,int targetY,float* ang);//ボム軌道を計算する
	__int8 DrawCursor();
};

#endif