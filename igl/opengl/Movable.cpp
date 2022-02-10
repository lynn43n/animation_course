#include "Movable.h"
#include <iostream>
Movable::Movable()
{
	Tout = Eigen::Affine3d::Identity();
	Tin = Eigen::Affine3d::Identity();
	parent = NULL;
}

Movable::Movable(const Movable& mov)
{
	Tout = mov.Tout;
	Tin = mov.Tin;
}

Eigen::Matrix4f Movable::MakeTransScale()
{
	return (Tout.matrix() * Tin.matrix()).cast<float>();
}

Eigen::Matrix4d Movable::MakeTransScaled()
{
	return (Tout.matrix() * Tin.matrix());
}

Eigen::Matrix4d Movable::MakeTransd()
{
	Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
	mat.col(3) << Tin.translation(), 1;

	return (Tout.matrix() * mat);
}
Eigen::Matrix4d Movable::ParentTrans() {
	Eigen::Matrix4d ans = Eigen::Matrix4d::Identity();
	Movable* currentP = parent;
	while (currentP != NULL) {
		ans = currentP->MakeTransScaled() * ans;
		currentP = currentP->parent;
	}
	return ans;
}

void Movable::setCenterOfRot(Eigen::Vector3d new_center)
{
	Tin.pretranslate(-new_center);
	Tout.pretranslate(new_center);
}

//this method should not be in movable class
void Movable::setParent(Movable* parentRef)
{
	this->parent = parentRef;

}

//this method should not be in movable class
Movable* Movable::getParent()
{
	return this->parent;
}
void Movable::MyTranslate(Eigen::Vector3d amt, bool preRotation)
{
	if (preRotation) {
		Tout.pretranslate(amt);
	}
	else
		Tout.translate(amt);
}
//angle in radians
void Movable::MyRotate(Eigen::Vector3d rotAxis, double angle)
{
	Tout.rotate(Eigen::AngleAxisd(angle, rotAxis));
}

void Movable::MyRotate(const Eigen::Matrix3d& rot)
{
	Tout.rotate(rot);
}

void Movable::MyRotate(Eigen::Vector3d rotAxis, double angle, bool yAxis)
{
	Eigen::Matrix3d mat = Tout.rotation().matrix();
	mat.transposeInPlace();
	if (yAxis)
		Tout.rotate(Eigen::AngleAxisd(angle, mat * rotAxis));
	else
		Tout.rotate(Eigen::AngleAxisd(angle, rotAxis));
}


void Movable::MyScale(Eigen::Vector3d amt)
{
	Tin.scale(amt);
}

void Movable::SetCenterOfRotation(Eigen::Vector3d amt)
{
	Tout.pretranslate(amt);
	Tin.pretranslate(-amt);


}
//to delete
Eigen::Vector3d Movable::getCenterOfRotationTout()
{
	return -Tout.translation();
}

Eigen::Vector3d Movable::GetCenterOfRotation() {
	return -Tin.translation();
}

void Movable::MyTranslateInSystem(Eigen::Matrix3d rot, Eigen::Vector3d amt)
{
	Tout.pretranslate(rot.transpose() * amt);
}

void Movable::RotateInSystem(Eigen::Vector3d rotAxis, double angle)
{
	Tout.rotate(Eigen::AngleAxisd(angle, Tout.rotation().transpose() *
		rotAxis.normalized())); //we will multiply the vector with the rotate matrix

}



























//void Movable::TranslateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d amt, bool preRotation)
//{
//	Eigen::Vector3d v = Mat.transpose().block<3, 3>(0, 0) * amt; //transpose instead of inverse
//	MyTranslate(v, preRotation);
//}
//
//void Movable::RotateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d rotAxis, double angle)
//{
//	Eigen::Vector3d v = Mat.transpose().block<3, 3>(0, 0) * rotAxis; //transpose instead of inverse
//	MyRotate(v.normalized(), angle);
//}
//
//
//void Movable::SetCenterOfRotation(Eigen::Vector3d amt)
//{
//	Tout.pretranslate(Tout.rotation().matrix().block<3, 3>(0, 0) * amt);
//	Tin.translate(-amt);
//}
//
//Eigen::Vector3d Movable::GetCenterOfRotation()
//{
//	return Tin.translation();
//}
