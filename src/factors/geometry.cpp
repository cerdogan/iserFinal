/**
 * @file geometry.cpp
 * @author Can Erdogan
 * @date OCt 14, 2013
 * @brief This file contains the data structures to represent the meshes of the objects.
 */

#include "geometry.h"
#include <iostream>

using namespace std;

/* ********************************************************************************************* */
void recurseParts (const aiScene* scene, aiNode* node, const Eigen::Matrix4d& T, 
		vector <Part>& object) {

	// Create the part
	static const bool debug = 1;
	if(node->mNumChildren == 0) {
		for(size_t mesh_idx = 0; mesh_idx < node->mNumMeshes; mesh_idx++) {
			Part part;
			part.mass = 1.0;
			part.mesh = scene->mMeshes[node->mMeshes[mesh_idx]];
			part.T = T;
			object.push_back(part);
		}
		return;
	}

	// Draw the children of this node
	for(size_t child_idx = 0; child_idx < node->mNumChildren; child_idx++) {

		// Get the child node
		aiNode* child = node->mChildren[child_idx];
		if(strstr(child->mName.C_Str(), "Light") != NULL) return;
		if(strstr(child->mName.C_Str(), "Camera") != NULL) return;

		// Get the transform between this node and the child node
		aiMatrix4x4 t = child->mTransformation;
		Eigen::Matrix4d Tnext;
		Tnext << t.a1, t.a2, t.a3, t.a4, t.b1, t.b2, t.b3, t.b4, t.c1, t.c2, t.c3, t.c4, t.d1, 
			t.d2, t.d3, t.d4;

		// Make the call
		recurseParts(scene, child, T * Tnext, object);
	}
}

/* ********************************************************************************************* */
void readPart (const string& filePath, vector <Part>& object) {

	// Read the object data
  Assimp::Importer* importer = new Assimp::Importer;
  const aiScene* scene = importer->ReadFile(filePath.c_str(), 
		aiProcess_JoinIdenticalVertices);
	recurseParts(scene, scene->mRootNode, Eigen::Matrix4d::Identity(), object);

	// Get the unique edges of each part for this object
	for(size_t i = 0; i < object.size(); i++) {
		object[i].name = filePath;
		aiMesh* mesh = object[i].mesh;
		for(size_t face_idx = 0; face_idx < mesh->mNumFaces; face_idx++) {
			aiFace& face = mesh->mFaces[face_idx];
			for(size_t edge_idx = 0; edge_idx < face.mNumIndices; edge_idx++) {
				size_t i1 = face.mIndices[edge_idx];
				size_t i2 = face.mIndices[(edge_idx+1) % face.mNumIndices];
				bool belongs = false;
				for(size_t found_idx = 0; found_idx < object[i].edges.size(); found_idx++) {
					Eigen::Vector2i edge = object[i].edges[found_idx];
					if(((i1 == edge(0)) && (i2 == edge(1))) || ((i1 == edge(1)) && (i2 == edge(0)))) {
						belongs = true;	
						break;
					}
				}
				if(!belongs) object[i].edges.push_back(Eigen::Vector2i(i1, i2));
			}
		}
	
	}
}

/* ********************************************************************************************* */
void readParts (std::vector <Part>& load, std::vector <Part>& lever, std::vector <Part>& fulcrum) {

  // Load the lever (concave) object with hardcoded values (see data/box-relative.txt for original)
  Assimp::Importer* importer1 = new Assimp::Importer;
  Assimp::Importer* importer2 = new Assimp::Importer;
  Assimp::Importer* importer3 = new Assimp::Importer;
  const aiScene* scene1 = importer1->ReadFile("../data/side.dae",
		aiProcess_JoinIdenticalVertices);
	Part p1, p2, p3, p4;
	p1.mesh = p2.mesh = p3.mesh = p4.mesh = scene1->mMeshes[0];
	p1.mass = p2.mass = p3.mass = p4.mass = 1.0;
	p1.bounded = p2.bounded = p3.bounded = p4.bounded = false;
	p1.T << 1,0,0,-0.119533,0,1,0,-0.0320665,0,0,1,0.0848614,0,0,0,1;
	p2.T << 1,0,0,-0.157633,0,1,0,0.221934,0,0,1,0.0848614,0,0,0,1;
	p3.T << 0,1,0,-0.190006,-1,0,0,0.189561,0,0,1,0.0848614,0,0,0,1;
	p4.T << 0,-1,0,0.102094,1,0,0,0.0384064,0,0,1,0.0848614,0,0,0,1;
	load.push_back(p1);
	load.push_back(p2);
	load.push_back(p3);
	load.push_back(p4);

	// Load the lever (convex) object
  const aiScene* scene2 = importer2->ReadFile("../data/stick.dae",
		aiProcess_JoinIdenticalVertices);
	Part p5;
	p5.mesh = scene2->mMeshes[0];
	p5.mass = 1.0;
	assert(p5.mesh->mNumVertices > 0);
	p5.bounded = false;
	p5.T = Eigen::Matrix4d::Identity();
	lever.push_back(p5);

	// Load the fulcrum (convex) object
  const aiScene* scene3 = importer3->ReadFile("../data/fulcrum.dae",
		aiProcess_JoinIdenticalVertices);
	Part p6;
	p6.mass = 1.0;
	p6.mesh = scene3->mMeshes[0];
	assert(p6.mesh->mNumVertices > 0);
	p6.bounded = false;
	p6.T = Eigen::Matrix4d::Identity();
	Eigen::Vector3d fcom (0.0, 0.0, 0.0);
	for(size_t i = 0; i < 7; i++) {
		if(i == 5) continue;
		aiVector3D av = p6.mesh->mVertices[i];
		Eigen::Vector4d vLocal (av.x, av.y, av.z, 1.0);
		Eigen::Vector4d vT = p6.T * vLocal;
		Eigen::Vector3d v (vT(0), vT(1), vT(2));
		fcom += v;
		cout << i << ": " <<  v.transpose() << endl;
	}
	p6.com = fcom / 6.0;
	fulcrum.push_back(p6);
}

