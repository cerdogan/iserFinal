/**
 * @file geometry.h
 * @author Can Erdogan
 * @date OCt 14, 2013
 * @brief This file contains the data structures to represent the meshes of the objects.
 */

#pragma once

#include <Eigen/Dense>
#include <assimp/Importer.hpp>     
#include <assimp/scene.h>           
#include <assimp/postprocess.h>     
#include <vector>
#include <iostream>

/* ********************************************************************************************* */
/// Object definition hardcoded

struct Object {
	std::vector <std::vector <size_t> > faces;
	std::vector <Eigen::Vector3d> vertices;
};

/* ********************************************************************************************* */
/// Constants to identify the chosen face and edges

namespace ex1 {

// Faces
static const size_t fulcrum_base_face = 0;
static const size_t lever_input_face = 6, lever_load_face = 7;
static const size_t lever_fulcrum_face = 2;
static const size_t l_base_face = 11; // base has multiple parts

// Edges
static const size_t l_edge_face = 3, l_edge_id = 3; // base has multiple parts!
static const size_t f_edge_face = 2, f_edge_id = 3;

};

/* ********************************************************************************************* */
/// The convex part definition
struct Part {
	aiMesh* mesh;				///< The mesh of the part with vertex locations expressed in the local frame
	Eigen::Matrix4d T;	///< The transformation between the object frame and the local frame
	std::vector <Eigen::Vector2i> edges;	///< The unique edges in the part
	bool bounded;
	Eigen::Vector3d com;
	double mass;
	std::string name;

	/// Returns the position of one of the vertices in the part frame
	Eigen::Vector3d getVertexW (int index) const {
		aiVector3D av = mesh->mVertices[index];
		Eigen::Vector4d vLocal (av.x, av.y, av.z, 1.0);
		Eigen::Vector4d vT = T * vLocal;
		return Eigen::Vector3d (vT(0), vT(1), vT(2));
	}

	/// Returns the position of one of the vertices in the part frame
	Eigen::Vector3d getVertexW (int face_idx, int index) const {
		const aiFace& face = mesh->mFaces[face_idx];
		aiVector3D av = mesh->mVertices[face.mIndices[index % face.mNumIndices]];
		Eigen::Vector4d vLocal (av.x, av.y, av.z, 1.0);
		Eigen::Vector4d vT = T * vLocal;
		return Eigen::Vector3d (vT(0), vT(1), vT(2));
	}

	/// Returns the length of an edge
	double getLength (int face_idx, int edge_idx) const {
		Eigen::Vector3d v1 = getVertexW(face_idx, edge_idx);
		Eigen::Vector3d v2 = getVertexW(face_idx, edge_idx+1);
		return (v1-v2).norm();
	}

	/// Returns the equation for the plane that is defined by the given face
	Eigen::Vector4d getPlane (int face_idx) const {
		Eigen::Vector3d v1 = getVertexW(face_idx, 0);
		Eigen::Vector3d v2 = getVertexW(face_idx, 1);
		Eigen::Vector3d v3 = getVertexW(face_idx, 2);
		Eigen::Vector3d v21 = v2 - v1, v31 = v3 - v1;
		Eigen::Vector3d vn = (v21.cross(v31)).normalized();
		return Eigen::Vector4d(vn(0), vn(1), vn(2), -v1.dot(vn));
	}
	
	/// Returns the outer normal to the edge
	Eigen::Vector4d getSidePlane(int face_idx, int edge_idx) const {
		Eigen::Vector3d v1 = getVertexW(face_idx, edge_idx);
		Eigen::Vector3d v2 = getVertexW(face_idx, edge_idx + 1);
		Eigen::Vector3d v3 = getVertexW(face_idx, edge_idx + 2);
		Eigen::Vector3d v21n = (v2 - v1).normalized(), v31 = v3 - v1;
		Eigen::Vector3d v31b = v31.dot(v21n) * v21n;
		Eigen::Vector3d n = -(v31 - v31b);
		double d = -v1.dot(n);
		return Eigen::Vector4d(n(0), n(1), n(2), d);
	}

	/// Returns the edge plane - that is the center is at the given vertex and the normal
	/// is along direction of the edge, either towards or away from the neighbor vector
	Eigen::Vector4d getEdgePlane(int face_idx, int v1idx, int v2idx) const {
		Eigen::Vector3d v1 = getVertexW(face_idx, v1idx);
		Eigen::Vector3d v2 = getVertexW(face_idx, v2idx);
		Eigen::Vector3d n = (v2 - v1).normalized();
		double d = -v1.dot(n);
		return Eigen::Vector4d(n(0), n(1), n(2), d);
	}
};

/* ********************************************************************************************* */
/// Read the parts
void readParts (std::vector <Part>& load, std::vector <Part>& lever, std::vector <Part>& fulcrum); 

/// Read the part
void readPart (const std::string& filePath, std::vector <Part>& object);

/// Creates the list of parts of this object (scene)
void recurseParts (const aiScene* scene, aiNode* node, const Eigen::Matrix4d& T, 
		std::vector <Part>& object);
