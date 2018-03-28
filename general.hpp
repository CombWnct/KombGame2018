#ifndef KOMB_GAME_GENERAL_GENERAL_HPP_21_MAR
#define KOMB_GAME_GENERAL_GENERAL_HPP_21_MAR

#include "ggTools3.hpp"

int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int);
namespace general {

	__int8 LoadModelData(std::vector<ggTools3::meshData>* mesh, std::string PAMOD);//���f����ǂݍ���

	__int8 InitDxLib();//dxlib�������������
	__int8 InitBullet();//bullet�������������
	__int8 Init();//���O��Ԃ�������
	__int8 CheckKey();
	__int8 CheckBullets();//�e�ۂ��`�F�b�N
	__int8 CreateTarget();//�I���쐬
	__int8 CreateBlocks();//�u���b�N���쐬
	__int8 DrawTarget();
	__int8 DrawBlocks();
	__int8 CheckBombs();
	__int8 CalcBomb(int targetX,int targetY,float* ang);//�{���O�����v�Z����
	__int8 DrawCursor();
};

#endif