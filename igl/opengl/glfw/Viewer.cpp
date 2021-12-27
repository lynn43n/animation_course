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

// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;


namespace igl
{
    namespace opengl
    {
        namespace glfw
        {

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
                link_num(0),
                destination(Eigen::Vector3d(5, 0, 0)),
                ikAnimation(false),
                fabricAnimation(false),
                link_length(1.6)
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

            void Viewer::SetNewShape(int savedIndex) {
                parents.push_back(-1);
                data_list.back().set_visible(false, 1);

                data_list.back().set_visible(true, 2);
                data_list.back().show_faces = 3;
                selected_data_index = savedIndex;
                int last_index = data_list.size() - 1;
                if (last_index > 1)
                    parents[last_index] = last_index - 1;
            }

            double Viewer::legalRange(double num) {
                if (num > 1)
                    num = 1;
                if (num < -1)
                    num = -1;
                return num;
            }

            void Viewer::printTip() {
                int lastLinkidx = link_num;
                tip = CalcParentsTrans(lastLinkidx) *data(lastLinkidx).MakeTransd() *
                    Eigen::Vector4d(data(lastLinkidx).V.colwise().mean()[0], data(lastLinkidx).V.colwise().maxCoeff()[1],data(lastLinkidx).V.colwise().mean()[2], 1);
                std::cout << "tip: (" << tip.head(3).transpose() << ")" << std::endl;
            }
            void Viewer::printDestination() {
                destination = data(0).MakeTransd().col(3).head(3);
                std::cout << "destination: (" << destination.transpose() << ")" << std::endl;
            }
            void Viewer::printP() {
                int idx = selected_data_index;
                Eigen::Matrix3d mat = idx == -1 ? MakeTransd().block(0, 0, 3, 3) : data().MakeTransd().block(0, 0, 3, 3);
                std::cout << "rotation of " << idx << ": " << std::endl;
                std::cout << mat << std::endl;
            }

            IGL_INLINE Eigen::Matrix4d Viewer::MakeParentTrans(int mesh_id) {
                if (parents[mesh_id] == -1 || parents[mesh_id] == mesh_id)
                    return Eigen::Transform<double, 3, Eigen::Affine>::Identity().matrix();
                Eigen::Matrix4d t = data_list[parents[mesh_id]].MakeTransd();
                Eigen::Matrix4d tp = MakeParentTrans(parents[mesh_id]);
                return MakeParentTrans(parents[mesh_id]) * data_list[parents[mesh_id]].MakeTransd();
            }

            IGL_INLINE Eigen::Matrix3d Viewer::GetParentsRotationInverse(int index) {

                Eigen::Matrix3d parentsInverse = data(index).GetRotation().inverse();
                int i = parents[index];
                while (i != -1)
                {
                    parentsInverse = parentsInverse * data(i).GetRotation().inverse();
                    i = parents[i];
                }

                return parentsInverse;
            }


            IGL_INLINE void Viewer::fabrik_solver() {
               //todo  http://devguis.com/chapter-13-implementing-inverse-kinematics-hands-on-c-game-animation-programming.html

                Eigen::Vector3d start_position = data(1).MakeTransd().block(0, 1, 3, 3).col(2);
                Eigen::Vector3d goal = data(0).MakeTransd().block(0, 1, 3, 3).col(2);
                int last_index = data_list.size() - 1;
                double link_size = 1.6;                
                double size = link_num;
                double mThreashold = 0.1;

                //check first if we are unable to reach the ball
                if ((goal - start_position).norm() > link_size * size) {
                    fabricAnimation = false;
                    std::cout << "cannot reach" << std::endl;
                    return;
                }
                std::vector<ViewerData> mIKChain;


                 
                // Iterate backwards
                if (size > 0) {
                    //mWorldChain[size - 1] = goal;

                }
                for (int i = size - 2; i >= 0; i--) {

                }

                // Iterate forwards
                if (size > 0) {

                    //mWorldChain[0] = base;
                }
                for (int i = 1; i < size; i++) {

                }


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


                //for (unsigned int i = 0; i<plugins.size(); ++i)
                //  if (plugins[i]->post_load())
                //    return true;


                if (mesh_file_name_string != "C:/Users/alina/source/repos/EngineForAnimationCourse/tutorial/data/sphere.obj") {
                    data().MyTranslateInSystem(data().GetRotation(), Eigen::RowVector3d(0, 0, 1.6));
                    data().kd_tree.init(data().V, data().F);
                    data().drawAxis(data().kd_tree.m_box);
                    data().SetCenterOfRotation(Eigen::RowVector3d(0, 0, -0.8));

                    int lastLinkidx = link_num;
                    tip = CalcParentsTrans(lastLinkidx) *
                        data(lastLinkidx).MakeTransd() *
                        Eigen::Vector4d(data(lastLinkidx).V.colwise().mean()[0], data(lastLinkidx).V.colwise().maxCoeff()[1], data(lastLinkidx).V.colwise().mean()[2], 1);
                    
                    link_num++;
                    SetNewShape(link_num);
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


            IGL_INLINE Eigen::Matrix3d Viewer::CalcParentsInverseRotation(int index) {
                Eigen::Matrix3d rot = data(index).GetRotation().inverse();

                for (int i = index - 1; i > 0; --i)
                    rot *= data(i).GetRotation().inverse();

                return rot;
            }


            Eigen::Matrix4d Viewer::CalcParentsTrans(int indx)
            {
                Eigen::Matrix4d prevTrans = Eigen::Matrix4d::Identity();
                for (int i = indx; parents[i] >= 0; i = parents[i])
                {
                    prevTrans = data_list[parents[i]].MakeTransd() * prevTrans;
                }
                return prevTrans;
            }

            IGL_INLINE void Viewer::togleCCD() {
                if (data_list.size() > 1) {
                    Eigen::Vector4d root = data_list[1].MakeTransd() * Eigen::Vector4d(0, 0, -link_length / 2, 1);
                    Eigen::Vector4d ball = data_list[0].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1);
                    double dist = (root - ball).norm();
                    
                    //first check if we are able to reach 
                    if (link_num * link_length < dist) {
                        std::cout << "cannot reach" << std::endl;
                        isActive = false;
                    }
                    else {
                        SetAnimation();
                    }
                }
                else {
                    std::cout << "cannot reach" << std::endl;
                    isActive = false;
                }
            }

            IGL_INLINE void Viewer::animateCCD() {
                Eigen::Vector4d ball = data_list[0].MakeTransd() * Eigen::Vector4d(0, 0, 0, 1),
                    E, R, RE, RD;
                Eigen::Vector3d cross;
                double dist = 0.0;

                for (int i = link_num; i > 0; --i) {
                    E = CalcParentsTrans(link_num) * data_list[link_num].MakeTransd() * Eigen::Vector4d(0, 0, link_length / 2, 1);
                    dist = (E - ball).norm();
                    R = CalcParentsTrans(i) * data_list[i].MakeTransd() * Eigen::Vector4d(0, 0, -link_length / 2, 1);
                    RE = E - R;
                    RD = ball - R;

                    double dot_product = RD.normalized().dot(RE.normalized());
                    double alpha = acos(dot_product > 1 ? 1 : dot_product < -1 ? -1 : dot_product);
                    cross = Eigen::Vector3d(RE[0], RE[1], RE[2]).cross(Eigen::Vector3d(RD[0], RD[1], RD[2])).normalized();
                    cross = CalcParentsInverseRotation(i) * cross;
                    data_list[i].MyRotate(cross, alpha / 30);
                }

                if (dist < 0.1)
                    isActive = false;
            }

        } // end namespace
    } // end namespace
}
