#include "Common/Common.h"
#include "Demos/Visualization/MiniGL.h"
#include "Demos/Visualization/Selection.h"
#include "Simulation/TimeManager.h"
#include <Eigen/Dense>
#include "Simulation/SimulationModel.h"
#include "Simulation/TimeStepController.h"
#include <iostream>
#include "Demos/Visualization/Visualization.h"
#include "Simulation/DistanceFieldCollisionDetection.h"
#include "Utils/OBJLoader.h"
#include "Utils/Logger.h"
#include "Utils/Timing.h"
#include "Utils/FileSystem.h"
#include "Demos/Common/DemoBase.h"
#include "Demos/Common/TweakBarParameters.h"
#include "Simulation/Simulation.h"

// Enable memory leak detection
#if defined(_DEBUG) && !defined(EIGEN_ALIGN)
	#define new DEBUG_NEW 
#endif

using namespace PBD;
using namespace Eigen;
using namespace std;
using namespace Utilities;

void timeStep ();
void buildModel ();
void createMesh();
void render ();
void reset();
void TW_CALL setStiffness(const void* value, void* clientData);
void TW_CALL getStiffness(void* value, void* clientData);
void TW_CALL setPoissonRatio(const void* value, void* clientData);
void TW_CALL getPoissonRatio(void* value, void* clientData);
void TW_CALL setVolumeStiffness(const void* value, void* clientData);
void TW_CALL getVolumeStiffness(void* value, void* clientData);
void TW_CALL setNormalizeStretch(const void* value, void* clientData);
void TW_CALL getNormalizeStretch(void* value, void* clientData);
void TW_CALL setNormalizeShear(const void* value, void* clientData);
void TW_CALL getNormalizeShear(void* value, void* clientData);
void TW_CALL setSimulationMethod(const void *value, void *clientData);
void TW_CALL getSimulationMethod(void *value, void *clientData);
void TW_CALL setContactTolerance(const void *value, void *clientData);
void TW_CALL getContactTolerance(void *value, void *clientData);
void TW_CALL setContactStiffnessRigidBody(const void *value, void *clientData);
void TW_CALL getContactStiffnessRigidBody(void *value, void *clientData);
void TW_CALL setContactStiffnessParticleRigidBody(const void *value, void *clientData);
void TW_CALL getContactStiffnessParticleRigidBody(void *value, void *clientData);


DemoBase *base;
DistanceFieldCollisionDetection cd;

const unsigned int width = 30;
const unsigned int depth = 5;
const unsigned int height = 5; 
short simulationMethod = 2;
Real stiffness = 1.0;
Real poissonRatio = 0.3;
bool normalizeStretch = false;
bool normalizeShear = false;
Real volumeStiffness = 1.0;

// main 
int main( int argc, char **argv )
{
	REPORT_MEMORY_LEAKS

	base = new DemoBase();
	base->init(argc, argv, "Bar collision demo");

	SimulationModel *model = new SimulationModel();
	model->init();
	Simulation::getCurrent()->setModel(model);

	buildModel();

	base->createParameterGUI();

	// OpenGL
	MiniGL::setClientIdleFunc (timeStep);		
	MiniGL::addKeyFunc('r', reset);
	MiniGL::setClientSceneFunc(render);			
	MiniGL::setViewport (40.0f, 0.1f, 500.0f, Vector3r (5.0, 10.0, 30.0), Vector3r (5.0, 0.0, 0.0));

	TwType enumType2 = TwDefineEnum("SimulationMethodType", NULL, 0);
	TwAddVarCB(MiniGL::getTweakBar(), "SimulationMethod", enumType2, setSimulationMethod, getSimulationMethod, &simulationMethod,
			" label='Simulation method' enum='0 {None}, 1 {Volume constraints}, 2 {FEM based PBD}, 3 {Strain based dynamics (no inversion handling)}, 4 {Shape matching (no inversion handling)}, 5 {XPBD volume constraints}' group=Simulation");
	TwAddVarCB(MiniGL::getTweakBar(), "stiffness", TW_TYPE_REAL, setStiffness, getStiffness, model, " label='Stiffness'  min=0.0 step=0.1 precision=4 group='Solid' ");
	TwAddVarCB(MiniGL::getTweakBar(), "poissonRatio", TW_TYPE_REAL, setPoissonRatio, getPoissonRatio, model, " label='Poisson ratio'  min=0.0 step=0.1 precision=4 group='Solid' ");
	TwAddVarCB(MiniGL::getTweakBar(), "normalizeStretch", TW_TYPE_BOOL32, setNormalizeStretch, getNormalizeStretch, model, " label='Normalize stretch' group='Solid' ");
	TwAddVarCB(MiniGL::getTweakBar(), "normalizeShear", TW_TYPE_BOOL32, setNormalizeShear, getNormalizeShear, model, " label='Normalize shear' group='Solid' ");
	TwAddVarCB(MiniGL::getTweakBar(), "volumeStiffness", TW_TYPE_REAL, setVolumeStiffness, getVolumeStiffness, model, " label='Volume stiffness'  min=0.0 step=0.1 precision=4 group='Solid' ");
	TwAddVarCB(MiniGL::getTweakBar(), "ContactTolerance", TW_TYPE_REAL, setContactTolerance, getContactTolerance, &cd, " label='Contact tolerance'  min=0.0 step=0.001 precision=3 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "ContactStiffnessRigidBody", TW_TYPE_REAL, setContactStiffnessRigidBody, getContactStiffnessRigidBody, model, " label='Contact stiffness RB'  min=0.0 step=0.1 precision=2 group=Simulation ");
	TwAddVarCB(MiniGL::getTweakBar(), "ContactStiffnessParticleRigidBody", TW_TYPE_REAL, setContactStiffnessParticleRigidBody, getContactStiffnessParticleRigidBody, model, " label='Contact stiffness Particle-RB'  min=0.0 step=0.1 precision=2 group=Simulation ");

	MiniGL::mainLoop ();	

	base->cleanup();

	Utilities::Timing::printAverageTimes();
	Utilities::Timing::printTimeSums();

	delete Simulation::getCurrent();
	delete base;
	delete model;

	return 0;
}

void reset()
{
	Utilities::Timing::printAverageTimes();
	Utilities::Timing::reset();

	Simulation::getCurrent()->reset();
	base->getSelectedParticles().clear();

	Simulation::getCurrent()->getModel()->cleanup();
	Simulation::getCurrent()->getTimeStep()->getCollisionDetection()->cleanup();

	buildModel();
}

void timeStep ()
{
	const Real pauseAt = base->getValue<Real>(DemoBase::PAUSE_AT);
	if ((pauseAt > 0.0) && (pauseAt < TimeManager::getCurrent()->getTime()))
		base->setValue(DemoBase::PAUSE, true);

	if (base->getValue<bool>(DemoBase::PAUSE))
		return;

	// Simulation code
	SimulationModel *model = Simulation::getCurrent()->getModel();
	const unsigned int numSteps = base->getValue<unsigned int>(DemoBase::NUM_STEPS_PER_RENDER);
	for (unsigned int i = 0; i < numSteps; i++)
	{
		START_TIMING("SimStep");
		Simulation::getCurrent()->getTimeStep()->step(*model);
		STOP_TIMING_AVG;
	}

	for (unsigned int i = 0; i < model->getTetModels().size(); i++)
		model->getTetModels()[i]->updateMeshNormals(model->getParticles());
}

void loadObj(const std::string &filename, VertexData &vd, IndexedFaceMesh &mesh, const Vector3r &scale)
{
	std::vector<OBJLoader::Vec3f> x;
	std::vector<OBJLoader::Vec3f> normals;
	std::vector<OBJLoader::Vec2f> texCoords;
	std::vector<MeshFaceIndices> faces;
	OBJLoader::Vec3f s = { (float)scale[0], (float)scale[1], (float)scale[2] };
	OBJLoader::loadObj(filename, &x, &faces, &normals, &texCoords, s);

	mesh.release();
	const unsigned int nPoints = (unsigned int)x.size();
	const unsigned int nFaces = (unsigned int)faces.size();
	const unsigned int nTexCoords = (unsigned int)texCoords.size();
	mesh.initMesh(nPoints, nFaces * 2, nFaces);
	vd.reserve(nPoints);
	for (unsigned int i = 0; i < nPoints; i++)
	{
		vd.addVertex(Vector3r(x[i][0], x[i][1], x[i][2]));
	}
	for (unsigned int i = 0; i < nTexCoords; i++)
	{
		mesh.addUV(texCoords[i][0], texCoords[i][1]);
	}
	for (unsigned int i = 0; i < nFaces; i++)
	{
		// Reduce the indices by one
		int posIndices[3];
		int texIndices[3];
		for (int j = 0; j < 3; j++)
		{
			posIndices[j] = faces[i].posIndices[j] - 1;
			if (nTexCoords > 0)
			{
				texIndices[j] = faces[i].texIndices[j] - 1;
				mesh.addUVIndex(texIndices[j]);
			}
		}

		mesh.addFace(&posIndices[0]);
	}
	mesh.buildNeighbors();

	mesh.updateNormals(vd, 0);
	mesh.updateVertexNormals(vd);

	LOG_INFO << "Number of triangles: " << nFaces;
	LOG_INFO << "Number of vertices: " << nPoints;
}

void buildModel ()
{
	TimeManager::getCurrent ()->setTimeStepSize (static_cast<Real>(0.005));
	SimulationModel *model = Simulation::getCurrent()->getModel();

	createMesh();

	// create static rigid body
	string fileName = FileSystem::normalizePath(base->getExePath() + "/resources/models/cube.obj");
	IndexedFaceMesh mesh;
	VertexData vd;
	loadObj(fileName, vd, mesh, Vector3r::Ones());	
	mesh.setFlatShading(true);

	string fileNameTorus = FileSystem::normalizePath(base->getExePath() + "/resources/models/torus.obj");
	IndexedFaceMesh meshTorus;
	VertexData vdTorus;
	loadObj(fileNameTorus, vdTorus, meshTorus, Vector3r::Ones());

	SimulationModel::RigidBodyVector &rb = model->getRigidBodies();
	rb.resize(2);

	// floor
	rb[0] = new RigidBody();
	rb[0]->initBody(1.0,
		Vector3r(0.0, -5.5, 0.0),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vd, mesh,
		Vector3r(100.0, 1.0, 100.0));
	rb[0]->setMass(0.0);

	// torus
	rb[1] = new RigidBody();
	rb[1]->initBody(1.0,
		Vector3r(5.0, -1.5, 0.0),
		Quaternionr(1.0, 0.0, 0.0, 0.0),
		vdTorus, meshTorus,
		Vector3r(3.0, 3.0, 3.0));
	rb[1]->setMass(0.0);
	rb[1]->setFrictionCoeff(static_cast<Real>(0.1));

	Simulation::getCurrent()->getTimeStep()->setCollisionDetection(*model, &cd);
	cd.setTolerance(static_cast<Real>(0.05));
	
	const std::vector<Vector3r> &vertices1 = rb[0]->getGeometry().getVertexDataLocal().getVertices();
	const unsigned int nVert1 = static_cast<unsigned int>(vertices1.size());
	cd.addCollisionBox(0, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices1.data(), nVert1, Vector3r(100.0, 1.0, 100.0));

	const std::vector<Vector3r> &vertices2 = rb[1]->getGeometry().getVertexDataLocal().getVertices();
	const unsigned int nVert2 = static_cast<unsigned int>(vertices2.size());
	cd.addCollisionTorus(1, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, vertices2.data(), nVert2, Vector2r(3.0, 1.5));

	SimulationModel::TetModelVector &tm = model->getTetModels();
	ParticleData &pd = model->getParticles();
	for (unsigned int i = 0; i < tm.size(); i++)
	{
		const unsigned int nVert = tm[i]->getParticleMesh().numVertices();
		unsigned int offset = tm[i]->getIndexOffset();
		tm[i]->setFrictionCoeff(static_cast<Real>(0.1));
		cd.addCollisionObjectWithoutGeometry(i, CollisionDetection::CollisionObject::TetModelCollisionObjectType, &pd.getPosition(offset), nVert, true);
	}
}


void render ()
{
	base->render();
}


void createMesh()
{
	SimulationModel* model = Simulation::getCurrent()->getModel();
	model->addRegularTetModel(width, height, depth,
		Vector3r(4.5, 3, 0), Matrix3r::Identity(), Vector3r(9.0, 1.5, 1.5));

	// init constraints
	stiffness = 1.0;
	if (simulationMethod == 5)
		stiffness = 100000;

	volumeStiffness = 1.0;
	if (simulationMethod == 5)
		volumeStiffness = 100000;

	ParticleData& pd = model->getParticles();
	for (unsigned int cm = 0; cm < model->getTetModels().size(); cm++)
	{
		model->addSolidConstraints(model->getTetModels()[cm], simulationMethod, stiffness, 
			poissonRatio, volumeStiffness, normalizeStretch, normalizeShear);

		model->getTetModels()[cm]->updateMeshNormals(pd);

		LOG_INFO << "Number of tets: " << model->getTetModels()[cm]->getParticleMesh().numTets();
		LOG_INFO << "Number of vertices: " << width * height * depth;
	}
}


void TW_CALL setStiffness(const void* value, void* clientData)
{
	stiffness = *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<FEMTetConstraint, Real, &FEMTetConstraint::m_stiffness>(stiffness);
	((SimulationModel*)clientData)->setConstraintValue<StrainTetConstraint, Real, &StrainTetConstraint::m_stretchStiffness>(stiffness);
	((SimulationModel*)clientData)->setConstraintValue<StrainTetConstraint, Real, &StrainTetConstraint::m_shearStiffness>(stiffness);
	((SimulationModel*)clientData)->setConstraintValue<DistanceConstraint, Real, &DistanceConstraint::m_stiffness>(stiffness);
	((SimulationModel*)clientData)->setConstraintValue<DistanceConstraint_XPBD, Real, &DistanceConstraint_XPBD::m_stiffness>(stiffness);
	((SimulationModel*)clientData)->setConstraintValue<ShapeMatchingConstraint, Real, &ShapeMatchingConstraint::m_stiffness>(stiffness);
}

void TW_CALL getStiffness(void* value, void* clientData)
{
	*(Real*)(value) = stiffness;
}

void TW_CALL setVolumeStiffness(const void* value, void* clientData)
{
	volumeStiffness = *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<VolumeConstraint, Real, &VolumeConstraint::m_stiffness>(volumeStiffness);
	((SimulationModel*)clientData)->setConstraintValue<VolumeConstraint_XPBD, Real, &VolumeConstraint_XPBD::m_stiffness>(volumeStiffness);
}

void TW_CALL getVolumeStiffness(void* value, void* clientData)
{
	*(Real*)(value) = volumeStiffness;
}

void TW_CALL getPoissonRatio(void* value, void* clientData)
{
	*(Real*)(value) = poissonRatio;
}

void TW_CALL setPoissonRatio(const void* value, void* clientData)
{
	poissonRatio = *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<FEMTetConstraint, Real, &FEMTetConstraint::m_poissonRatio>(poissonRatio);
}

void TW_CALL getNormalizeStretch(void* value, void* clientData)
{
	*(bool*)(value) = normalizeStretch;
}

void TW_CALL setNormalizeStretch(const void* value, void* clientData)
{
	normalizeStretch = *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<StrainTetConstraint, bool, &StrainTetConstraint::m_normalizeStretch>(normalizeStretch);
}

void TW_CALL getNormalizeShear(void* value, void* clientData)
{
	*(bool*)(value) = normalizeShear;
}

void TW_CALL setNormalizeShear(const void* value, void* clientData)
{
	normalizeShear = *(const Real*)(value);
	((SimulationModel*)clientData)->setConstraintValue<StrainTetConstraint, bool, &StrainTetConstraint::m_normalizeShear>(normalizeShear);
}

void TW_CALL setSimulationMethod(const void *value, void *clientData)
{
	const short val = *(const short *)(value);
	*((short*)clientData) = val;
	reset();
}

void TW_CALL getSimulationMethod(void *value, void *clientData)
{
	*(short *)(value) = *((short*)clientData);
}

void TW_CALL setContactStiffnessRigidBody(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((SimulationModel*)clientData)->setContactStiffnessRigidBody(val);
}

void TW_CALL getContactStiffnessRigidBody(void *value, void *clientData)
{
	*(Real *)(value) = ((SimulationModel*)clientData)->getContactStiffnessRigidBody();
}

void TW_CALL setContactStiffnessParticleRigidBody(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((SimulationModel*)clientData)->setContactStiffnessParticleRigidBody(val);
}

void TW_CALL getContactStiffnessParticleRigidBody(void *value, void *clientData)
{
	*(Real *)(value) = ((SimulationModel*)clientData)->getContactStiffnessParticleRigidBody();
}

void TW_CALL setContactTolerance(const void *value, void *clientData)
{
	const Real val = *(const Real *)(value);
	((DistanceFieldCollisionDetection*)clientData)->setTolerance(val);
}

void TW_CALL getContactTolerance(void *value, void *clientData)
{
	*(Real *)(value) = ((DistanceFieldCollisionDetection*)clientData)->getTolerance();
}
