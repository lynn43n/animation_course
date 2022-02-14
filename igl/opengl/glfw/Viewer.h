// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_OPENGL_GLFW_VIEWER_H
#define IGL_OPENGL_GLFW_VIEWER_H

#ifndef IGL_OPENGL_4
#define IGL_OPENGL_4
#endif

#include "../../igl_inline.h"
#include "../MeshGL.h"

#include "../ViewerData.h"
#include "ViewerPlugin.h"


#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <string>
#include <cstdint>

#define IGL_MOD_SHIFT           0x0001
#define IGL_MOD_CONTROL         0x0002
#define IGL_MOD_ALT             0x0004
#define IGL_MOD_SUPER           0x0008


//Ass1 comment
typedef std::set<std::pair<double, int>> PriorityQueue;
//end comment


namespace igl
{
    namespace opengl
    {
        namespace glfw
        {
            // GLFW-based mesh viewer
            class Viewer : public Movable
            {
                //Ass3
                double delta = 0.1;
                double maxDistance;
                //end Ass3

            public:
               
                virtual void Init(const std::string config);
                virtual void Animate() {}
                virtual void WhenTranslate() {}
                virtual Eigen::Vector3d GetCameraPosition() { return Eigen::Vector3d(0, 0, 0); }
                virtual Eigen::Vector3d GetCameraForward() { return Eigen::Vector3d(0, 0, -1); }
                virtual Eigen::Vector3d GetCameraUp() { return Eigen::Vector3d(0, 1, 0); }

                Viewer();
                virtual ~Viewer();
                // Mesh IO
                IGL_INLINE bool load_mesh_from_file(const std::string& mesh_file_name);
                IGL_INLINE bool save_mesh_to_file(const std::string& mesh_file_name);

                // Scene IO
                IGL_INLINE bool load_scene();
                IGL_INLINE bool load_scene(std::string fname);
                IGL_INLINE bool save_scene();
                IGL_INLINE bool save_scene(std::string fname);


                IGL_INLINE void open_dialog_load_mesh();
                IGL_INLINE void open_dialog_save_mesh();

                IGL_INLINE void draw() {}
                IGL_INLINE void init_target_object(int savedIndex);

                //ASS1 comment help function mesh simplification
                void comp_obj_quad_error();
                void caseInverible(Eigen::Vector4d& p1, Eigen::Matrix4d& qtag, Eigen::Vector3d& p2);
                void caseNotInvertible(Eigen::Vector3d& p1, const Eigen::MatrixXd& V, int v1, int v2, Eigen::Vector4d& p2);
                void comp_opt_cost_position(const int e, const Eigen::MatrixXd& V, double& cost, Eigen::Vector3d& p);
                double formula_cost(Eigen::Vector4d p_cost, Eigen::Matrix4d q12);
                bool new_collapse_edge(Eigen::MatrixXd& V, Eigen::MatrixXi& F);
                bool check_existance(int neighbor_index, Eigen::MatrixXi& F);
                void initMeshdata();
                void meshSimplification(double num_iter); // simplify mesh

                //end comment Ass1

                //Ass 2 comment
                //Eigen::Vector3d moveDir = Eigen::Vector3d(-0.004, 0, 0);//initial of velocity*dircetion = (0.004*(-1,0,0))= (-0.004, 0, 0)
                // (-1,0,0) moving the movable object to left side of X's Axe.
                int moving_index = 0;
                void initTreesAndDrawForCollision();
                void setMovingButton();
                void checkCollision();//main checkeer
                bool recursiveCheckCollision(igl::AABB<Eigen::MatrixXd, 3>* node1, igl::AABB<Eigen::MatrixXd, 3>* node2, int i);//recursive checker,called by main checker
                bool checkTermsForBoxesCollision(Eigen::AlignedBox<double, 3>& box1, Eigen::AlignedBox<double, 3>& box2, int i);//check 15 terms for boxes collision, called by recursive checker
                //end comment Ass 2

                //Ass 3
                int link_num;
                Eigen::RowVector4d tip_position;
                Eigen::RowVector3d destination_position;
                bool ikAnimation;
                int current_picked;
                //ASS3:
                Eigen::Matrix4d ParentsTrans_mat4d(int index);
                Eigen::Matrix3d ParentsInvRot_mat3d(int index);
                void IKSimulation();
                void toggleIKSimulation();
                void axisFixer();
                double distance_2Points(Eigen::Vector3d p1, Eigen::Vector3d p2);
                void FabrikAlgo();
                void transformfab(std::vector<Eigen::Vector3d>& p, std::vector<Eigen::Vector3d>& oldp);
                Eigen::Vector3d getTarget();
                Eigen::Vector3d getTipbyindex(int index);
                // end Ass3

                ////////////////////////
                // Multi-mesh methods //
                ////////////////////////

                // Return the current mesh, or the mesh corresponding to a given unique identifier
                //
                // Inputs:
                //   mesh_id  unique identifier associated to the desired mesh (current mesh if -1)
                IGL_INLINE ViewerData& data(int mesh_id = -1);
                IGL_INLINE const ViewerData& data(int mesh_id = -1) const;

                // Append a new "slot" for a mesh (i.e., create empty entries at the end of
                // the data_list and opengl_state_list.
                //
                // Inputs:
                //   visible  If true, the new mesh is set to be visible on all existing viewports
                // Returns the id of the last appended mesh
                //
                // Side Effects:
                //   selected_data_index is set this newly created, last entry (i.e.,
                //   #meshes-1)
                IGL_INLINE int append_mesh(bool visible = true);

                // Erase a mesh (i.e., its corresponding data and state entires in data_list
                // and opengl_state_list)
                //
                // Inputs:
                //   index  index of mesh to erase
                // Returns whether erasure was successful <=> cannot erase last mesh
                //
                // Side Effects:
                //   If selected_data_index is greater than or equal to index then it is
                //   decremented
                // Example:
                //   // Erase all mesh slots except first and clear remaining mesh
                //   viewer.selected_data_index = viewer.data_list.size()-1;
                //   while(viewer.erase_mesh(viewer.selected_data_index)){};
                //   viewer.data().clear();
                //
                IGL_INLINE bool erase_mesh(const size_t index);

                // Retrieve mesh index from its unique identifier
                // Returns 0 if not found
                IGL_INLINE size_t mesh_index(const int id) const;

                Eigen::Matrix4d CalcParentsTrans(int indx);
                inline bool SetAnimation() { return isActive = !isActive; }
            public:
                //////////////////////
                // Member variables //
                //////////////////////

                // Alec: I call this data_list instead of just data to avoid confusion with
                // old "data" variable.
                // Stores all the data that should be visualized
                std::vector<ViewerData> data_list;

                std::vector<int> parents;

                size_t selected_data_index;
                int next_data_id;
                bool isPicked;
                bool isActive;
               
                //project comment
                
                void Viewer::Initialize_Tree(int index);


                void Viewer::updateScore();
                void Viewer::showCorrectMenu();

                bool isResume = false;
                bool isGameStarted = false;
                int score = 0;
                int level = 1;
                int targetScore = 2;// start target score
                bool isCollisionTarget;
                bool isCollisionSnake;
                bool start;
                bool isNextLevel;
                bool gameLost;

                int snake_size;
                bool snake_view;
                float prev_tic;
                IGL_INLINE void move_targets(int level);

                IGL_INLINE void generate_target(int level);
              
                float scroll_position;

            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };

        } // end namespace
    } // end namespace
} // end namespace

#ifndef IGL_STATIC_LIBRARY
#  include "Viewer.cpp"
#endif

#endif