#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>
#include <Eigen/dense>


class Movable
{
public:
	Movable();
	Movable(const Movable& mov);
	Eigen::Matrix4f MakeTransScale(); //delete
	Eigen::Matrix4d MakeTransd();
	Eigen::Matrix4d ParentTrans();
	void setParent(Movable* parentRef);
	Movable* getParent();
	Eigen::Matrix4d MakeTransScaled();
	void MyTranslate(Eigen::Vector3d amt, bool preRotation);
	void MyRotate(Eigen::Vector3d rotAxis, double angle); 
	void MyRotate(const Eigen::Matrix3d& rot);
	void MyRotate(Eigen::Vector3d rotAxis, double angle, bool yAxis);
	void MyScale(Eigen::Vector3d amt);



	void SetCenterOfRotation(Eigen::Vector3d amt);
	Eigen::Vector3d getCenterOfRotationTout();
	Eigen::Vector3d GetCenterOfRotation();
	Eigen::Vector3d getCenterOfRotation();
	void setCenterOfRot(Eigen::Vector3d new_center);
	void MyTranslateInSystem(Eigen::Matrix3d rot, Eigen::Vector3d amt);
	void RotateInSystem(Eigen::Vector3d rotAxis, double angle);


	Eigen::Matrix3d GetRotation() const { return Tout.rotation().matrix(); }
	Eigen::Affine3d GetTin() const { return Tin; }
	Eigen::Affine3d GetTout() const { return Tout; }

	virtual ~Movable() {}
	Eigen::Affine3d Tout, Tin;
	Movable* parent;
private:
	
};

