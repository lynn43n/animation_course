#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>
#include <Eigen/dense>


class Movable
{
public:
	Eigen::Affine3d Tout, Tin;
	

	Movable();
	Movable(const Movable& mov);
	Eigen::Matrix4f MakeTransScale();
	Eigen::Matrix4d MakeTransd();
	Eigen::Matrix4d MakeTransScaled();
	void MyTranslate(Eigen::Vector3d amt, bool preRotation);
	void MyRotate(Eigen::Vector3d rotAxis, double angle);
	void MyRotate(const Eigen::Matrix3d& rot);
	void MyRotate(Eigen::Vector3d rotAxis, double angle, bool zAxis);
	// project
	Eigen::Quaterniond GetRotationQ();
	void Movable::RotateInSystem(Eigen::Matrix4d Mat, Eigen::Vector3d rotAxis, double angle);
	Eigen::Vector3d Movable::GetCenter();

	Eigen::Vector3d GetTranslation() const { return (Tout * Tin).matrix().block(0, 3, 3, 1); }

	void MyScale(Eigen::Vector3d amt);

	void SetCenterOfRotation(Eigen::Vector3d amt);

	void MyTranslateInSystem(Eigen::Matrix3d rot, Eigen::Vector3d amt);

	void RotateInSystem(Eigen::Vector3d rotAxis, double angle);

	void Movable::MyRotate(const Eigen::Quaterniond rot);
	void Movable::RotateInSystem(const Eigen::Matrix3d& mat, const Eigen::Quaterniond rot);
	

	Eigen::Matrix3d GetRotation() const { return Tout.rotation().matrix(); }

	virtual ~Movable() {}
private:
	//Eigen::Affine3d Tout,Tin;
};
