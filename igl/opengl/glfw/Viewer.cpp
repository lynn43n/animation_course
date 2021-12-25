// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

//#include <chrono>
#include <thread>

#include <Eigen/LU>


#include <cmath>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cassert>

#include <igl/project.h>
//#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>


#include <igl/edge_flaps.h>
#include <igl/shortest_edge_and_midpoint.h>

// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;
using namespace std;


namespace igl
{
    namespace opengl
    {
        namespace glfw
        {

            typedef std::set<std::pair<double, int> > PriorityQueue;

            void Viewer::Init(const std::string config)
            {


            }



            IGL_INLINE Viewer::Viewer() :
                data_list(1),
                selected_data_index(0),
                next_data_id(1),
                isPicked(false),
                isActive(false),
                tip(Eigen::Vector4d(0, 0, 0, 0)),
                links_number(0),
                destination(Eigen::Vector3d(5, 0, 0))
            {
                data_list.front().id = 0;



                // Temporary variables initialization
               // down = false;
              //  hack_never_moved = true;
                scroll_position = 0.0f;

                // Per face
                data().set_face_based(false);


#ifndef IGL_VIEWER_VIEWER_QUIET
                const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  1,2     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
                );
                std::cout << usage << std::endl;
#endif
            }

            IGL_INLINE Viewer::~Viewer()
            {
            }



            void Viewer::toggleIK() {
                if (IKon == true) {
                    fixAxis();
                }
                IKon = !IKon;
            }

            void Viewer::animateIK() {
                Eigen::Vector4d root4 = data_list[1].MakeTransScaled() * Eigen::Vector4d(0, -0.8, 0, 1);
                Eigen::Vector3d root = Eigen::Vector3d(root4[0], root4[1], root4[2]);

                Eigen::Vector4d ball4 = data_list[0].MakeTransScaled() * Eigen::Vector4d(0, 0, 0, 1);
                Eigen::Vector3d ball = Eigen::Vector3d(ball4[0], ball4[1], ball4[2]);

                double dist = (root - ball).norm();

                if (dist > 6.4) { //6.4 is arm length fully extended
                    cout << "cannot reach" << endl;
                    IKon = false;
                    return;
                }
                Eigen::Vector4d E4;
                Eigen::Vector3d E;
                for (int i = links_number; i > 0; i--) {
                    E4 = ParentsTrans(links_number) * data_list[links_number].MakeTransScaled() * Eigen::Vector4d(0, 0.8, 0, 1);
                    E = Eigen::Vector3d(E4[0], E4[1], E4[2]);
                    dist = (E - ball).norm();

                    Eigen::Vector4d R4 = ParentsTrans(i) * data_list[i].MakeTransScaled() * Eigen::Vector4d(0, -0.8, 0, 1);
                    Eigen::Vector3d R = Eigen::Vector3d(R4[0], R4[1], R4[2]);

                    Eigen::Vector3d RE = E - R;
                    Eigen::Vector3d RD = ball - R;

                    float dot = RD.normalized().dot(RE.normalized());
                    float alphaRad = acosf(dot); //alpah in radians
                    if (dist > 0.3)
                        alphaRad = alphaRad / 20;
                    if (dot >= 1.0)
                        alphaRad = 0;

                    Eigen::Vector3d cros = RE.cross(RD);
                    cros.normalize();
                    cros = ParentsInverseRot(i) * cros;
                    data_list[i].MyRotate(cros, alphaRad, false);
                    // ----- Debug Prints ----
                    //float alpha =  alphaRad / M_PI * 180.0; //alpha in degrees
                    //cout << "R: " << endl << R << endl << "E: " << endl << E << endl;
                    //cout << "RE: " << endl << RE << endl << "RD: " << endl << RD << endl;
                    //cout << "alpha: " << alphaRad << endl;
                    //cout << "dot: " << dot << endl;
                }
                E4 = ParentsTrans(links_number) * data_list[4].MakeTransScaled() * Eigen::Vector4d(0, 0.8, 0, 1);
                E = Eigen::Vector3d(E4[0], E4[1], E4[2]);
                dist = (E - ball).norm();
                if (dist < 0.1 || IKon == false) {
                    IKon = false;
                    fixAxis();
                }
                cout << "Distance: " << dist << endl;
            }

            void Viewer::fixAxis() {
                float firstY = 0;
                for (int i = 1; i <= links_number; i++) {
                    Eigen::Matrix3d RU = data_list[i].GetTout().rotation().matrix();
                    if (RU(1, 1) < 1.0) {
                        if (RU(1, 1) > -1.0) {
                            float y = atan2f(RU(1, 0), -RU(1, 2));
                            data_list[i].MyRotate(Eigen::Vector3d(0, 1, 0), -y, false);
                            if (i != links_number) {
                                data_list[i + 1].MyRotate(Eigen::Vector3d(0, 1, 0), y, true);
                            }
                        }
                    }
                }
            }



            Eigen::Matrix3d Viewer::ParentsInverseRot(int index) {
                Eigen::Matrix3d rot = data(index).GetTout().rotation().matrix().inverse();
                int i = index - 1;
                while (i > 0) {
                    rot = rot * data(i).GetTout().rotation().matrix().inverse();
                    i--;
                }
                return rot;
            }


            Eigen::Matrix4d Viewer::ParentsTrans(int index) {
                if (index <= 1)
                    return Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix();
                return ParentsTrans(index - 1) * data_list[index - 1].MakeTransScaled();
            }



            void Viewer::printTipPos() {
                for (int i = 1; i <= links_number; i++) {
                    Eigen::Vector4d pos4 = ParentsTrans(i) * data_list[i].MakeTransScaled() * Eigen::Vector4d(0, 0.8, 0, 1);
                    Eigen::Vector3d pos3 = Eigen::Vector3d(pos4[0], pos4[1], pos4[2]);
                    cout << "----- Tip " << i << " -----" << endl << pos3 << endl << "-----------------" << endl;
                }
            }

            void Viewer::printRotation() {
                if (selected_data_index == -1)
                    cout << "Rotation: " << endl << GetTout().rotation().matrix() << endl;
                else cout << "Rotation: " << endl << data_list[selected_data_index].GetTout().rotation().matrix() << endl;
            }

            void Viewer::printBallPos() {
                Eigen::Vector4d pos4 = data_list[0].MakeTransScaled() * Eigen::Vector4d(0, 0, 0, 1);
                Eigen::Vector3d pos3 = Eigen::Vector3d(pos4[0], pos4[1], pos4[2]);
                cout << "-- Destination --" << endl << pos3 << endl << "-----------------" << endl;
            }


            IGL_INLINE bool Viewer::load_mesh_from_file(
                const std::string& mesh_file_name_string)
            {

                // Create new data slot and set to selected
                if (!(data().F.rows() == 0 && data().V.rows() == 0))
                {
                    append_mesh();
                }
                data().clear();

                size_t last_dot = mesh_file_name_string.rfind('.');
                if (last_dot == std::string::npos)
                {
                    std::cerr << "Error: No file extension found in " <<
                        mesh_file_name_string << std::endl;
                    return false;
                }

                std::string extension = mesh_file_name_string.substr(last_dot + 1);

                if (extension == "off" || extension == "OFF")
                {
                    Eigen::MatrixXd V;
                    Eigen::MatrixXi F;
                    if (!igl::readOFF(mesh_file_name_string, V, F))
                        return false;
                    data().set_mesh(V, F);
                }
                else if (extension == "obj" || extension == "OBJ")
                {
                    Eigen::MatrixXd corner_normals;
                    Eigen::MatrixXi fNormIndices;

                    Eigen::MatrixXd UV_V;
                    Eigen::MatrixXi UV_F;
                    Eigen::MatrixXd V;
                    Eigen::MatrixXi F;

                    if (!(
                        igl::readOBJ(
                            mesh_file_name_string,
                            V, UV_V, corner_normals, F, UV_F, fNormIndices)))
                    {
                        return false;
                    }

                    data().set_mesh(V, F);
                    if (UV_V.rows() > 0)
                    {
                        data().set_uv(UV_V, UV_F);
                    }

                }
                else
                {
                    // unrecognized file type
                    printf("Error: %s is not a recognized file type.\n", extension.c_str());
                    return false;
                }

                data().compute_normals();
                data().uniform_colors(Eigen::Vector3d(51.0 / 255.0, 43.0 / 255.0, 33.3 / 255.0),
                    Eigen::Vector3d(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0),
                    Eigen::Vector3d(255.0 / 255.0, 235.0 / 255.0, 80.0 / 255.0));

                // Alec: why?
                if (data().V_uv.rows() == 0)
                {
                    data().grid_texture();
                }

                //initilize mesh for decimation
                //data().V_clone = data().V;
                //data().F_clone = data().F;
                //data().init_mesh();

                //for (unsigned int i = 0; i<plugins.size(); ++i)
                //  if (plugins[i]->post_load())
                //    return true;


                if (mesh_file_name_string != "C:/Users/alina/source/repos/EngineForAnimationCourse/tutorial/data/sphere.obj") {

                    data().MyTranslateInSystem(data().GetRotation(), Eigen::RowVector3d(0, 0, 1.6));
                    data().kd_tree.init(data().V, data().F);
                    data().drawAxis(data().kd_tree.m_box);
                    data().SetCenterOfRotation(Eigen::RowVector3d(0, 0, -0.8));
                    std::cout << "HILYNN\n" << std::endl;
                    int lastLinkidx = links_number;
                    tip = CalcParentsTrans(lastLinkidx) *
                        data(lastLinkidx).MakeTransd() *
                        Eigen::Vector4d(data(lastLinkidx).V.colwise().mean()[0], data(lastLinkidx).V.colwise().maxCoeff()[1], data(lastLinkidx).V.colwise().mean()[2], 1);


                    links_number++;
                    if (links_number == 1)
                        data_list[1].setParent(nullptr);
                    if (links_number > 1) {
                        Movable* parent = &data_list[links_number - 1];
                        data_list[links_number].setParent(parent);
                        //data_list[i].ParentTrans();
                    }

                }

                return true;
            }

            IGL_INLINE bool Viewer::save_mesh_to_file(
                const std::string& mesh_file_name_string)
            {
                // first try to load it with a plugin
                //for (unsigned int i = 0; i<plugins.size(); ++i)
                //  if (plugins[i]->save(mesh_file_name_string))
                //    return true;

                size_t last_dot = mesh_file_name_string.rfind('.');
                if (last_dot == std::string::npos)
                {
                    // No file type determined
                    std::cerr << "Error: No file extension found in " <<
                        mesh_file_name_string << std::endl;
                    return false;
                }
                std::string extension = mesh_file_name_string.substr(last_dot + 1);
                if (extension == "off" || extension == "OFF")
                {
                    return igl::writeOFF(
                        mesh_file_name_string, data().V, data().F);
                }
                else if (extension == "obj" || extension == "OBJ")
                {
                    Eigen::MatrixXd corner_normals;
                    Eigen::MatrixXi fNormIndices;

                    Eigen::MatrixXd UV_V;
                    Eigen::MatrixXi UV_F;

                    return igl::writeOBJ(mesh_file_name_string,
                        data().V,
                        data().F,
                        corner_normals, fNormIndices, UV_V, UV_F);
                }
                else
                {
                    // unrecognized file type
                    printf("Error: %s is not a recognized file type.\n", extension.c_str());
                    return false;
                }
                return true;
            }

            IGL_INLINE bool Viewer::load_scene()
            {
                std::string fname = igl::file_dialog_open();
                if (fname.length() == 0)
                    return false;
                return load_scene(fname);
            }

            IGL_INLINE bool Viewer::load_scene(std::string fname)
            {
                // igl::deserialize(core(),"Core",fname.c_str());
                igl::deserialize(data(), "Data", fname.c_str());
                return true;
            }

            IGL_INLINE bool Viewer::save_scene()
            {
                std::string fname = igl::file_dialog_save();
                if (fname.length() == 0)
                    return false;
                return save_scene(fname);
            }

            IGL_INLINE bool Viewer::save_scene(std::string fname)
            {
                //igl::serialize(core(),"Core",fname.c_str(),true);
                igl::serialize(data(), "Data", fname.c_str());

                return true;
            }

            IGL_INLINE void Viewer::open_dialog_load_mesh()
            {
                this->load_mesh_from_file("C:/Users/alina/source/repos/EngineForAnimationCourse/tutorial/data/zcylinder.obj");


            }

            IGL_INLINE void Viewer::open_dialog_save_mesh()
            {
                std::string fname = igl::file_dialog_save();

                if (fname.length() == 0)
                    return;

                this->save_mesh_to_file(fname.c_str());
            }

            IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
            {
                assert(!data_list.empty() && "data_list should never be empty");
                int index;
                if (mesh_id == -1)
                    index = selected_data_index;
                else
                    index = mesh_index(mesh_id);

                assert((index >= 0 && index < data_list.size()) &&
                    "selected_data_index or mesh_id should be in bounds");
                return data_list[index];
            }

            IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
            {
                assert(!data_list.empty() && "data_list should never be empty");
                int index;
                if (mesh_id == -1)
                    index = selected_data_index;
                else
                    index = mesh_index(mesh_id);

                assert((index >= 0 && index < data_list.size()) &&
                    "selected_data_index or mesh_id should be in bounds");
                return data_list[index];
            }

            IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
            {
                assert(data_list.size() >= 1);

                data_list.emplace_back();
                selected_data_index = data_list.size() - 1;
                data_list.back().id = next_data_id++;
                //if (visible)
                //    for (int i = 0; i < core_list.size(); i++)
                //        data_list.back().set_visible(true, core_list[i].id);
                //else
                //    data_list.back().is_visible = 0;
                return data_list.back().id;
            }

            IGL_INLINE bool Viewer::erase_mesh(const size_t index)
            {
                assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
                assert(data_list.size() >= 1);
                if (data_list.size() == 1)
                {
                    // Cannot remove last mesh
                    return false;
                }
                data_list[index].meshgl.free();
                data_list.erase(data_list.begin() + index);
                if (selected_data_index >= index && selected_data_index > 0)
                {
                    selected_data_index--;
                }

                return true;
            }

            IGL_INLINE size_t Viewer::mesh_index(const int id) const {
                for (size_t i = 0; i < data_list.size(); ++i)
                {
                    if (data_list[i].id == id)
                        return i;
                }
                return 0;
            }

            Eigen::Matrix4d Viewer::CalcParentsTrans(int indx)
            {
                Eigen::Matrix4d prevTrans = Eigen::Matrix4d::Identity();

                for (int i = indx; parents[i] >= 0; i = parents[i])
                {
                    //std::cout << "parent matrix:\n" << scn->data_list[scn->parents[i]].MakeTrans() << std::endl;
                    prevTrans = data_list[parents[i]].MakeTransd() * prevTrans;
                }

                return prevTrans;
            }
            IGL_INLINE void Viewer::IKSolver() {

                Eigen::Vector4d spherePos = (data_list[0].MakeTransScaled() * Eigen::Vector4d(0, 0, 0, 1));
                float cylinderLength = 1.6;
                float sphereLength = 0.6;
                int numberOfCylinder = (data_list.size() - 1);
                int baseCylinder = 1;
                float minDis = cylinderLength * numberOfCylinder + sphereLength;
                // Top = the top point of the topest cylinder
                for (int i = numberOfCylinder; i > 0; i--) {
                    Eigen::Vector4d topICylinder = data_list[i].ParentTrans() * data_list[i].MakeTransScaled() * Eigen::Vector4d(0, 0, 0.8, 1);
                    Eigen::Vector4d botICylinder = data_list[i].ParentTrans() * data_list[i].MakeTransScaled() * Eigen::Vector4d(0, 0, -0.8, 1);
                    Eigen::Vector4d topTopCylinder = data_list[numberOfCylinder].ParentTrans() * data_list[4].MakeTransScaled() * Eigen::Vector4d(0, 0, 0.8, 1);
                    Eigen::Vector4d botBaseCylinder = data_list[baseCylinder].ParentTrans() * data_list[baseCylinder].MakeTransScaled() * Eigen::Vector4d(0, 0, -0.8, 1);

                    float disBaseSphere = (botBaseCylinder - spherePos).norm();
                    float disIShpere = (topTopCylinder - spherePos).norm();
                    std::cout << disIShpere << std::endl;

                    if (disBaseSphere > minDis || disIShpere < 0.1)
                        is_IKSolver = false;

                    Eigen::Vector4d disTop_BotI = (topTopCylinder - botICylinder).normalized();
                    Eigen::Vector4d disSphere_BotI = (spherePos - botICylinder).normalized();

                    double cosAngle = disTop_BotI.dot(disSphere_BotI);
                    if (cosAngle > 1) {
                        cosAngle = 1;
                    }
                    if (cosAngle < -1) {
                        cosAngle = -1;
                    }
                    double angle_I_sphere = static_cast<double>(acos(cosAngle));

                    Eigen::Vector3d RE;
                    Eigen::Vector3d RD;
                    RE << disTop_BotI(0), disTop_BotI(1), disTop_BotI(2);
                    RD << disSphere_BotI(0), disSphere_BotI(1), disSphere_BotI(2);
                    Eigen::Vector3d rotationAxis = (RE.cross(RD)).normalized();
                    data_list[i].MyRotate(rotationAxis, angle_I_sphere);
                }


            }


        } // end namespace
    } // end namespace
}